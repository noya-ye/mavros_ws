#include "offboard_core_pkg/tasks/align_drop_snake_ego_task.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "offboard_core_pkg/mavros_iface.hpp"

namespace offboard_core_pkg {
namespace {

std::uint64_t nowUs(const rclcpp::Clock::SharedPtr &clock) {
  return static_cast<std::uint64_t>(
    clock->now().nanoseconds() / 1000ULL);
}

double pointSegmentDistance(
  double px, double py,
  double x0, double y0,
  double x1, double y1) {

  const double dx = x1 - x0;
  const double dy = y1 - y0;

  const double length_sq = dx * dx + dy * dy;

  if (length_sq < 1e-9) {
    return std::hypot(px - x0, py - y0);
  }

  const double t = std::clamp(
    ((px - x0) * dx + (py - y0) * dy) / length_sq,
    0.0,
    1.0);

  return std::hypot(
    px - (x0 + t * dx),
    py - (y0 + t * dy));
}

}  // namespace


AlignDropSnakeEgoTask::AlignDropSnakeEgoTask(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
  const Config &cfg)
: logger_(logger),
  clock_(std::move(clock)),
  cfg_(cfg),
  snake_(logger, cfg.snake),
  ego_(logger, clock_, std::move(goal_pub), cfg.ego),
  align_(
    cfg.align_pixels_per_meter,
    cfg.align_stable_frames,
    cfg.align_arrive_distance_m,
    cfg.align_max_step_m),
  red_cross_(
    cfg.red_cross_pixels_per_meter,
    cfg.red_cross_stable_frames,
    cfg.red_cross_arrive_distance_m,
    cfg.red_cross_max_step_m),
  down_drop_(
    logger,
    cfg.drop_height_m,
    cfg.drop_targets[0],
    cfg.drop_serial_device,
    cfg.drop_serial_baud_rate),
  land_(
    cfg.landing_timeout_s,
    cfg.landing_retry_interval_s) {}


std::string AlignDropSnakeEgoTask::name() const {
  return "ALIGN_DROP_SNAKE_EGO";
}


const char *AlignDropSnakeEgoTask::phaseName() const {

  switch (phase_) {

    case Phase::SNAKE:
      return "SNAKE";

    case Phase::BRAKING:
      return "BRAKING";

    case Phase::AVOIDING:
      return "AVOIDING";

    case Phase::ALIGNING:
      return "ALIGNING";

    case Phase::ALIGNING_RED:
      return "ALIGNING_RED";

    case Phase::DROPPING:
      return "DROPPING";

    case Phase::RETURNING:
      return "RETURNING";

    case Phase::LANDING:
      return "LANDING";

    case Phase::FINISHED:
      return "FINISHED";

    case Phase::FAILED:
      return "FAILED";
  }

  return "UNKNOWN";
}


// ============================================================================
// Task enter
// ============================================================================

void AlignDropSnakeEgoTask::onEnter(
  Context &ctx,
  MavrosIface &iface) {

  phase_ = Phase::SNAKE;

  avoidance_target_index_ = 0;
  avoidance_elapsed_s_ = 0.0;
  braking_elapsed_s_ = 0.0;
  braking_initial_speed_mps_ = 0.0;
  align_elapsed_s_ = 0.0;
  align_loss_elapsed_s_ = 0.0;
  alignment_entry_target_valid_ = false;
  lost_target_cooldown_s_ = 0.0;
  align_trigger_cooldown_s_ = 0.0;
  drop_target_index_ = 0;
  finish_after_return_ = false;

  failure_reason_.clear();

  alignment_latched_ = false;
  red_cross_completed_ = false;
  alignment_target_valid_ = false;
  completed_targets_enu_.clear();

  snake_.onEnter(ctx, iface);

  if (snake_.failed()) {

    phase_ = Phase::FAILED;

    ctx.fault = "snake route initialization failed";

    RCLCPP_ERROR(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] %s",
      ctx.fault.c_str());

    return;
  }

  RCLCPP_INFO(
    logger_,
    "[ALIGN_DROP_SNAKE_EGO] started");
}


// ============================================================================
// Main tick
// ============================================================================

ITask::Status AlignDropSnakeEgoTask::tick(
  Context &ctx,
  MavrosIface &iface,
  double dt_s) {

  const double dt = std::clamp(
    std::isfinite(dt_s) ? dt_s : 0.0,
    0.0,
    0.2);
  align_trigger_cooldown_s_ = std::max(0.0, align_trigger_cooldown_s_ - dt);

  // --------------------------------------------------------------------------
  // 独立状态
  // --------------------------------------------------------------------------

  if (phase_ == Phase::FAILED) {
    return Status::FAILURE;
  }
  if (phase_ == Phase::FINISHED) {
    return Status::SUCCESS;
  }

  if (phase_ == Phase::LANDING) {
    return tickLanding(ctx, iface, dt);
  }

  if (phase_ == Phase::ALIGNING ||
      phase_ == Phase::ALIGNING_RED ||
      phase_ == Phase::DROPPING) {
    return tickAlignment(ctx, iface, dt);
  }

  if (phase_ == Phase::RETURNING) {
    return tickReturn(ctx, iface, dt);
  }


  // --------------------------------------------------------------------------
  // 检测到下视 circle/contour 或 RedCross -> 触发对应纠偏
  // --------------------------------------------------------------------------

  lost_target_cooldown_s_ = std::max(0.0, lost_target_cooldown_s_ - dt);

  const bool red_cross_trigger =
    cfg_.red_cross_enabled &&
    !red_cross_completed_ &&
    redCrossFresh(ctx);
  const bool down_trigger =
    align_trigger_cooldown_s_ <= 0.0 &&
    (circleFresh(ctx) || contourFresh(ctx)) &&
    !insideCompletedDownTargetRadius(ctx);

  if (!alignment_latched_ && (red_cross_trigger || down_trigger)) {

    beginAlignment(
      ctx,
      iface,
      phase_ == Phase::AVOIDING
        ? ResumePhase::AVOIDING
        : ResumePhase::SNAKE,
      red_cross_trigger
        ? AlignmentSource::RED_CROSS
        : AlignmentSource::DOWN);

    return Status::RUNNING;
  }


  // ==========================================================================
  // SNAKE
  // ==========================================================================

  if (phase_ == Phase::SNAKE) {

    if (snake_.finished()) {

      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] snake route completed");

      return Status::SUCCESS;
    }


    SnakeGridTask::WaypointInfo waypoint;

    // ------------------------------------------------------------------------
    // 当前蛇形 waypoint 已经被占据
    // ------------------------------------------------------------------------

    while (
      snake_.waypointAt(
        snake_.currentIndex(),
        waypoint) &&
      occupiedAt(
        ctx,
        waypoint.x,
        waypoint.y)) {

      RCLCPP_WARN(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] skip occupied cell=(%d,%d)",
        waypoint.ix,
        waypoint.iy);

      snake_.skipCurrentWaypoint();
    }


    if (snake_.finished()) {

      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] "
        "snake route completed after occupied cells were skipped");

      return Status::SUCCESS;
    }


    if (!snake_.waypointAt(
          snake_.currentIndex(),
          waypoint)) {

      ctx.fault = "snake waypoint index is invalid";

      RCLCPP_ERROR(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] %s at index=%zu",
        ctx.fault.c_str(),
        snake_.currentIndex());

      return Status::FAILURE;
    }


    // ------------------------------------------------------------------------
    // 航段附近发现障碍 -> EGO
    // ------------------------------------------------------------------------

    if (
      obstacleDataFresh(ctx) &&
      obstacleNearSegment(
        ctx,
        ctx.position_enu.x,
        ctx.position_enu.y,
        waypoint.x,
        waypoint.y,
        &braking_obstacle_position_)) {

      if (!selectAvoidanceTarget(ctx)) {

        RCLCPP_ERROR(
          logger_,
          "[ALIGN_DROP_SNAKE_EGO] "
          "obstacle near waypoint=(%d,%d), "
          "no unoccupied avoidance target",
          waypoint.ix,
          waypoint.iy);

        failAndLand(
          ctx,
          iface,
          "no safe snake waypoint for EGO avoidance");

        return Status::RUNNING;
      }


      RCLCPP_WARN(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] "
        "obstacle detected; diverting from waypoint=(%d,%d) "
        "to index=%zu",
        waypoint.ix,
        waypoint.iy,
        avoidance_target_index_);


      beginAvoidance(ctx, iface);
      braking_start_position_ = ctx.position_enu;
      const double speed = std::hypot(
        ctx.velocity_enu.x, ctx.velocity_enu.y);
      braking_initial_speed_mps_ = std::isfinite(speed) ? speed : 0.0;
      if (braking_initial_speed_mps_ > 1e-3) {
        braking_direction_x_ =
          ctx.velocity_enu.x / braking_initial_speed_mps_;
        braking_direction_y_ =
          ctx.velocity_enu.y / braking_initial_speed_mps_;
      } else {
        braking_direction_x_ = 0.0;
        braking_direction_y_ = 0.0;
      }
      braking_elapsed_s_ = 0.0;
      phase_ = Phase::BRAKING;

      return Status::RUNNING;
    }


    // ------------------------------------------------------------------------
    // 正常 Snake
    // ------------------------------------------------------------------------

    const auto status =
      snake_.tick(ctx, iface, dt);

    if (status == Status::FAILURE) {

      failAndLand(
        ctx,
        iface,
        "snake task failed");

      return Status::RUNNING;
    }

    return status;
  }


  if (phase_ == Phase::BRAKING) {
    SnakeGridTask::WaypointInfo target;
    if (!snake_.waypointAt(avoidance_target_index_, target)) {
      phase_ = Phase::AVOIDING;
    } else {
      const double remaining = std::hypot(
        braking_obstacle_position_.x - ctx.position_enu.x,
        braking_obstacle_position_.y - ctx.position_enu.y);
      if (remaining <= std::max(0.0, cfg_.ego_handoff_distance_m)) {
        phase_ = Phase::AVOIDING;
        RCLCPP_INFO(
          logger_,
          "[ALIGN_DROP_SNAKE_EGO] braking-to-EGO handoff at "
          "obstacle distance %.2f m (limit %.2f m)",
          remaining, cfg_.ego_handoff_distance_m);
      } else {
        braking_elapsed_s_ += dt;
        const double delay = std::max(0.0, cfg_.braking_control_delay_s);
        const double deceleration = std::max(
          1e-3, cfg_.braking_deceleration_mps2);
        const double moving_time = std::max(0.0, braking_elapsed_s_ - delay);
        const double braking_time = std::min(
          moving_time,
          braking_initial_speed_mps_ / deceleration);
        const double delay_travel =
          braking_initial_speed_mps_ *
          std::min(braking_elapsed_s_, delay);
        const double stopping_distance =
          delay_travel +
          braking_initial_speed_mps_ * braking_time -
          0.5 * deceleration * braking_time * braking_time;
        const double kinematic_stop_distance =
          braking_initial_speed_mps_ * delay +
          braking_initial_speed_mps_ * braking_initial_speed_mps_ /
            (2.0 * deceleration);
        const double available_distance = std::max(
          0.0, remaining - std::max(0.0, cfg_.ego_handoff_distance_m));
        const double travel = std::clamp(
          stopping_distance, 0.0,
          std::min(kinematic_stop_distance, available_distance));
        ctx.position_setpoint_enu = {
          braking_start_position_.x + braking_direction_x_ * travel,
          braking_start_position_.y + braking_direction_y_ * travel,
          braking_start_position_.z};
        ctx.velocity_setpoint_enu = {0.0, 0.0, 0.0};
        ctx.acceleration_setpoint_enu = {0.0, 0.0, 0.0};
        ctx.setpoint_mode = SetpointMode::POSITION;
        ctx.use_position_velocity_acceleration = false;
        ctx.publish_position_setpoint = true;
        ctx.yaw_setpoint_enu = ctx.yaw_enu;
        return Status::RUNNING;
      }
    }
  }


  // ==========================================================================
  // EGO AVOIDING
  // ==========================================================================

  avoidance_elapsed_s_ += dt;

  if (avoidance_elapsed_s_ >=
      cfg_.avoidance_timeout_s) {

    RCLCPP_ERROR(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] "
      "avoidance timeout after %.1f s "
      "(limit %.1f s)",
      avoidance_elapsed_s_,
      cfg_.avoidance_timeout_s);

    failAndLand(
      ctx,
      iface,
      "EGO avoidance timed out");

    return Status::RUNNING;
  }


  const auto status =
    ego_.tick(ctx, iface, dt);


  if (status == Status::SUCCESS) {

    RCLCPP_INFO(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] "
      "EGO avoidance reached index=%zu; "
      "resuming snake route",
      avoidance_target_index_);


    ego_.onExit(ctx, iface);

    snake_.resumeAfterWaypoint(
      avoidance_target_index_,
      ctx);

    phase_ = Phase::SNAKE;

    avoidance_elapsed_s_ = 0.0;


  } else if (status == Status::FAILURE) {

    RCLCPP_ERROR(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] "
      "EGO avoidance task reported failure");

    failAndLand(
      ctx,
      iface,
      "EGO avoidance failed");
  }


  return Status::RUNNING;
}


// ============================================================================
// Begin alignment
// ============================================================================

void AlignDropSnakeEgoTask::beginAlignment(
  Context &ctx,
  MavrosIface &iface,
  ResumePhase resume_phase,
  AlignmentSource source) {

  // ==========================================================================
  // 修改 1：
  // 必须确认当前位置和 yaw 有效以后再保存返回点。
  // 避免把 NaN / invalid position 保存成 resume_position_。
  // ==========================================================================

  if (!ctx.position_valid ||
      !ctx.finitePosition() ||
      !std::isfinite(ctx.yaw_enu)) {

    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      1000,
      "[ALIGN_DROP_SNAKE_EGO] "
      "alignment trigger ignored because vehicle pose is invalid: "
      "position_valid=%d pos=(%.3f, %.3f, %.3f) yaw=%.3f",
      static_cast<int>(ctx.position_valid),
      ctx.position_enu.x,
      ctx.position_enu.y,
      ctx.position_enu.z,
      ctx.yaw_enu);

    return;
  }


  resume_phase_ = resume_phase;

  // --------------------------------------------------------------------------
  // 这里保存纠偏前的位置
  // --------------------------------------------------------------------------

  resume_position_ = ctx.position_enu;
  resume_yaw_ = ctx.yaw_enu;
  alignment_entry_target_valid_ = source == AlignmentSource::RED_CROSS
    ? redCrossPositionEnu(ctx, alignment_entry_target_)
    : (circlePositionEnu(ctx, alignment_entry_target_) ||
       contourPositionEnu(ctx, alignment_entry_target_));
  align_loss_elapsed_s_ = 0.0;


  RCLCPP_INFO(
    logger_,
    "[ALIGN_DROP_SNAKE_EGO] "
    "saving resume pose: "
    "position=(%.3f, %.3f, %.3f) yaw=%.3f",
    resume_position_.x,
    resume_position_.y,
    resume_position_.z,
    resume_yaw_);


  if (resume_phase_ == ResumePhase::SNAKE) {

    snake_.onPause(ctx, iface);

  } else {

    ego_.onExit(ctx, iface);
  }


  if (source == AlignmentSource::RED_CROSS) {
    red_cross_.onEnter(ctx, iface);
    phase_ = Phase::ALIGNING_RED;
  } else {
    align_.onEnter(ctx, iface);
    phase_ = Phase::ALIGNING;
  }

  alignment_target_valid_ = false;

  align_elapsed_s_ = 0.0;

  alignment_latched_ = true;

  RCLCPP_INFO(
    logger_,
    "[ALIGN_DROP_SNAKE_EGO] %s alignment triggered in %s",
    source == AlignmentSource::RED_CROSS ? "red-cross" : "down",
    resume_phase_ == ResumePhase::AVOIDING
      ? "EGO"
      : "snake");
}


// ============================================================================
// Alignment
// ============================================================================

ITask::Status AlignDropSnakeEgoTask::tickAlignment(
  Context &ctx,
  MavrosIface &iface,
  double dt_s) {

  if (phase_ == Phase::DROPPING) {
    const auto status = down_drop_.tick(ctx, iface, dt_s);
    if (status == Status::FAILURE) {
      down_drop_.onExit(ctx, iface);
      phase_ = Phase::FAILED;
      RCLCPP_ERROR(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] DownDrop failed: %s",
        ctx.fault.c_str());
      return Status::FAILURE;
    }
    if (status == Status::SUCCESS) {
      down_drop_.onExit(ctx, iface);
      finish_after_return_ = drop_target_index_ >= cfg_.drop_targets.size();
      phase_ = Phase::RETURNING;
      ctx.position_setpoint_enu = resume_position_;
      ctx.yaw_setpoint_enu = resume_yaw_;
      ctx.publish_position_setpoint = true;
      ctx.setpoint_mode = SetpointMode::POSITION;
      ctx.use_position_velocity_acceleration = false;
      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] drop completed; returning to alignment entry pose%s",
        finish_after_return_ ? " before task completion" : "");
    }
    return Status::RUNNING;
  }

  align_elapsed_s_ += dt_s;

  const bool red_cross_alignment = phase_ == Phase::ALIGNING_RED;
  const double timeout_s = red_cross_alignment
    ? cfg_.red_cross_timeout_s
    : cfg_.align_timeout_s;

  const bool detection_fresh = red_cross_alignment
    ? redCrossFresh(ctx)
    : (circleFresh(ctx) || contourFresh(ctx));
  align_loss_elapsed_s_ = detection_fresh ? 0.0 : align_loss_elapsed_s_ + dt_s;
  const bool detection_lost = align_loss_elapsed_s_ >= cfg_.align_loss_timeout_s;


  // --------------------------------------------------------------------------
  // 对准超时
  // --------------------------------------------------------------------------

  if (detection_lost || align_elapsed_s_ >= timeout_s) {

    // Abandon this attempt and return to the saved entry pose. Suppress this
    // target after returning so a still-fresh detection cannot retrigger it.
    Vec3 target;
    if (red_cross_alignment) {
      red_cross_.onExit(ctx, iface);
    } else {
      align_.onExit(ctx, iface);
      if (circlePositionEnu(ctx, target) ||
          contourPositionEnu(ctx, target)) {
        completed_targets_enu_.push_back(target);
      }
    }

    if (detection_lost && !red_cross_alignment && alignment_entry_target_valid_) {
      lost_target_ = alignment_entry_target_;
      // Count the cooldown only after return, while the route is running.
      lost_target_cooldown_s_ = cfg_.align_loss_retry_cooldown_s;
    }

    phase_ = Phase::RETURNING;

    ctx.position_setpoint_enu =
      resume_position_;

    ctx.yaw_setpoint_enu =
      resume_yaw_;

    ctx.publish_position_setpoint =
      true;

    ctx.setpoint_mode =
      SetpointMode::POSITION;

    ctx.use_position_velocity_acceleration =
      false;

    if (detection_lost) {
      RCLCPP_WARN(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] %s detection lost continuously for %.2f s; "
        "abandoning alignment after %.2f s and returning to the alignment entry pose",
        red_cross_alignment ? "red-cross" : "down",
        align_loss_elapsed_s_, align_elapsed_s_);
    } else {
      RCLCPP_WARN(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] %s alignment timed out after %.1f s; "
      "returning to the alignment entry pose",
      red_cross_alignment ? "red-cross" : "down",
      align_elapsed_s_);
    }

    return Status::RUNNING;
  }


  // --------------------------------------------------------------------------
  // 对准任务
  // --------------------------------------------------------------------------

  if (red_cross_alignment) {
    const auto status = red_cross_.tick(ctx, iface, dt_s);
    if (status == Status::FAILURE) {
      red_cross_.onExit(ctx, iface);
      ctx.fault = "red-cross alignment failed";
      phase_ = Phase::FAILED;
      RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] %s", ctx.fault.c_str());
      return Status::FAILURE;
    }
    if (status == Status::SUCCESS) {
      red_cross_.onExit(ctx, iface);
      red_cross_completed_ = true;
      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] red-cross alignment succeeded; "
        "further red-cross triggers disabled for this task run");
      alignment_latched_ = false;
      const auto &drop_target = cfg_.drop_targets[drop_target_index_];
      down_drop_.setTarget(drop_target);
      ++drop_target_index_;
      down_drop_.onEnter(ctx, iface);
      phase_ = Phase::DROPPING;
      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] starting DownDrop target=%d "
        "offset=(%.2f, %.2f)",
        drop_target.id, drop_target.offset_x, drop_target.offset_y);
    }
    return Status::RUNNING;
  }

  const auto status =
    align_.tick(ctx, iface, dt_s);

  if (
    align_.reached_arrival_tolerance() &&
    !alignment_target_valid_) {

    const bool has_circle_target =
      circlePositionEnu(ctx, alignment_target_enu_);

    const bool has_contour_target =
      !has_circle_target &&
      contourPositionEnu(ctx, alignment_target_enu_);

    if (!has_circle_target && !has_contour_target &&
        ctx.position_valid && ctx.finitePosition()) {
      alignment_target_enu_ = ctx.position_enu;
    }

    if (has_circle_target ||
        (ctx.position_valid && ctx.finitePosition())) {
      alignment_target_valid_ = true;
      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] captured %s target on arrival: "
        "ENU=(%.2f, %.2f, %.2f)",
        has_circle_target ? "circle" :
          (has_contour_target ? "contour" : "vehicle-position"),
        alignment_target_enu_.x,
        alignment_target_enu_.y,
        alignment_target_enu_.z);
    }
  }
  // 三重兜底：有 circle 目标、没有 circle 但有 contour 目标、没有任何目标但当前位置有效。用于去重


  if (status == Status::FAILURE) {

    align_.onExit(ctx, iface);

    ctx.fault =
      "down alignment failed";

    phase_ = Phase::FAILED;

    RCLCPP_ERROR(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] %s",
      ctx.fault.c_str());

    return Status::FAILURE;
  }


  // ==========================================================================
  // ALIGN 完成
  // ==========================================================================

  if (status == Status::SUCCESS) {

    if (phase_ == Phase::ALIGNING) {
      align_trigger_cooldown_s_ = cfg_.align_trigger_cooldown_s;
    }

    Vec3 target;


    // ------------------------------------------------------------------------
    // 保存已经处理过的目标位置，用于防止重复触发
    // ------------------------------------------------------------------------

    if (alignment_target_valid_) {
      target = alignment_target_enu_;
      completed_targets_enu_.push_back(target);

      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] alignment succeeded; "
        "target ENU=(%.2f, %.2f, %.2f)",
        target.x,
        target.y,
        target.z);

    } else if (circlePositionEnu(ctx, target)) {

      completed_targets_enu_.push_back(target);

      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] "
        "alignment succeeded; "
        "target ENU=(%.2f, %.2f, %.2f)",
        target.x,
        target.y,
        target.z);

    } else if (contourPositionEnu(ctx, target)) {

      completed_targets_enu_.push_back(target);

      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] alignment succeeded; "
        "target estimated from contour ENU=(%.2f, %.2f, %.2f)",
        target.x,
        target.y,
        target.z);

    } else {

      RCLCPP_WARN(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] "
        "alignment succeeded without a valid arrival position; "
        "retrigger suppression disabled");
    }


    alignment_latched_ = false;


    // ------------------------------------------------------------------------
    // 退出 AlignDownTask
    // ------------------------------------------------------------------------

    const bool target_confirmed = align_.target_confirmed();
    align_.onExit(ctx, iface);

    if (target_confirmed) {
      const auto &drop_target = cfg_.drop_targets[drop_target_index_];
      down_drop_.setTarget(drop_target);
      ++drop_target_index_;
      down_drop_.onEnter(ctx, iface);
      phase_ = Phase::DROPPING;
      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] AlignDown confirmed target; "
        "starting DownDrop target=%d offset=(%.2f, %.2f)",
        drop_target.id, drop_target.offset_x, drop_target.offset_y);
      return Status::RUNNING;
    }


    // =========================================================================
    // 修改 2：
    //
    // 不只是写 resume_position_。
    //
    // 在进入 RETURNING 的这一帧就明确恢复 POSITION 控制，
    // 不依赖下一帧 tickReturn() 再设置。
    // =========================================================================

    ctx.position_setpoint_enu =
      resume_position_;

    ctx.yaw_setpoint_enu =
      resume_yaw_;

    ctx.publish_position_setpoint =
      true;

    ctx.setpoint_mode =
      SetpointMode::POSITION;

    ctx.use_position_velocity_acceleration =
      false;


    phase_ =
      Phase::RETURNING;


    RCLCPP_INFO(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] "
      "returning to saved position="
      "(%.2f, %.2f, %.2f), yaw=%.3f",
      resume_position_.x,
      resume_position_.y,
      resume_position_.z,
      resume_yaw_);
  }


  return Status::RUNNING;
}


// ============================================================================
// RETURN TO PRE-ALIGN POSITION
// ============================================================================

ITask::Status AlignDropSnakeEgoTask::tickReturn(
  Context &ctx,
  MavrosIface &iface,
  double dt_s) {

  // ==========================================================================
  // 修改 3：
  //
  // 无论当前位置当前是否 valid，都不要让 POSITION 发布标志掉下去。
  //
  // 这样不会出现：
  //
  // position_valid=false
  //     ↓
  // return
  //     ↓
  // publish_position_setpoint 没有被重新设置
  //     ↓
  // 飞机永久停在纠偏位置
  //
  // ==========================================================================

  ctx.publish_position_setpoint =
    true;

  ctx.setpoint_mode =
    SetpointMode::POSITION;

  ctx.use_position_velocity_acceleration =
    false;

  ctx.yaw_setpoint_enu =
    resume_yaw_;


  // ==========================================================================
  // Position 暂时无效
  // ==========================================================================

  if (!ctx.position_valid ||
      !ctx.finitePosition()) {

    // 保持返回目标，不要静默什么都不做
    ctx.position_setpoint_enu =
      resume_position_;


    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      1000,
      "[RETURN] waiting for valid position: "
      "valid=%d current=(%.3f, %.3f, %.3f) "
      "saved=(%.3f, %.3f, %.3f)",
      static_cast<int>(ctx.position_valid),
      ctx.position_enu.x,
      ctx.position_enu.y,
      ctx.position_enu.z,
      resume_position_.x,
      resume_position_.y,
      resume_position_.z);


    return Status::RUNNING;
  }


  // ==========================================================================
  // 当前位置 -> 保存位置误差
  // ==========================================================================

  const double dx =
    resume_position_.x -
    ctx.position_enu.x;

  const double dy =
    resume_position_.y -
    ctx.position_enu.y;

  const double dz =
    resume_position_.z -
    ctx.position_enu.z;


  const double error_xy =
    std::hypot(dx, dy);

  const double error_z =
    std::abs(dz);

  const double distance =
    std::sqrt(
      dx * dx +
      dy * dy +
      dz * dz);

  RCLCPP_INFO_THROTTLE(
    logger_,
    *clock_,
    1000,
    "[RETURN] "
    "current=(%.3f, %.3f, %.3f) "
    "saved=(%.3f, %.3f, %.3f) "
    "error_xy=%.3f error_z=%.3f distance=%.3f",
    ctx.position_enu.x,
    ctx.position_enu.y,
    ctx.position_enu.z,
    resume_position_.x,
    resume_position_.y,
    resume_position_.z,
    error_xy,
    error_z,
    distance);


  // ==========================================================================
  // 修改 4：
  //
  // XY 和 Z 分开判断。
  //
  // 原代码：
  //
  // tolerance = min(arrive_xy_m, arrive_z_m)
  // distance_xyz <= tolerance
  //
  // 会让 Z 的小 tolerance 同时限制 XY。
  //
  // ==========================================================================

  const bool arrived_xy =
    error_xy <= cfg_.snake.arrive_xy_m;

  const bool arrived_z =
    error_z <= cfg_.snake.arrive_z_m;


  if (arrived_xy &&
      arrived_z) {

    // ------------------------------------------------------------------------
    // 最后明确保持在保存位置
    // ------------------------------------------------------------------------

    ctx.position_setpoint_enu =
      resume_position_;

    ctx.yaw_setpoint_enu =
      resume_yaw_;

    ctx.publish_position_setpoint =
      true;

    ctx.setpoint_mode =
      SetpointMode::POSITION;

    ctx.use_position_velocity_acceleration =
      false;

    alignment_latched_ = false;

    if (finish_after_return_) {
      phase_ = Phase::FINISHED;
      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] all configured DownDrop targets completed");
      return Status::SUCCESS;
    }


    // ========================================================================
    // 修改 5：
    // 返回完成后恢复被暂停的任务。
    // ========================================================================

    if (resume_phase_ ==
        ResumePhase::AVOIDING) {

      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] "
        "return completed; restarting EGO avoidance");

      ego_.onEnter(ctx, iface);

      phase_ =
        Phase::AVOIDING;

    } else {

      RCLCPP_INFO(
        logger_,
        "[ALIGN_DROP_SNAKE_EGO] "
        "return completed; resuming snake route");

      // Snake 在 beginAlignment() 中执行过 onPause()
      // 所以返回以后应恢复它。
      snake_.onResume(ctx, iface);

      phase_ =
        Phase::SNAKE;
    }


    RCLCPP_INFO(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] "
      "returned to saved position: "
      "error_xy=%.3f m error_z=%.3f m; "
      "resuming %s",
      error_xy,
      error_z,
      phase_ == Phase::AVOIDING
        ? "EGO avoidance"
        : "snake route");


    return Status::RUNNING;
  }


  // Keep commanding the pose captured when alignment began. The position
  // controller handles the full return trajectory.
  ctx.position_setpoint_enu =
    resume_position_;

  ctx.yaw_setpoint_enu =
    resume_yaw_;


  ctx.publish_position_setpoint =
    true;

  ctx.setpoint_mode =
    SetpointMode::POSITION;

  ctx.use_position_velocity_acceleration =
    false;


  return Status::RUNNING;
}


// ============================================================================
// Begin EGO avoidance
// ============================================================================

void AlignDropSnakeEgoTask::beginAvoidance(
  Context &ctx,
  MavrosIface &iface) {

  SnakeGridTask::WaypointInfo waypoint;


  snake_.waypointAt(
    avoidance_target_index_,
    waypoint);


  snake_.onPause(
    ctx,
    iface);


  ego_.setTargetEnu(
    waypoint.x,
    waypoint.y,
    waypoint.z);


  ego_.onEnter(
    ctx,
    iface);


  avoidance_elapsed_s_ =
    0.0;


  RCLCPP_INFO(
    logger_,
    "[ALIGN_DROP_SNAKE_EGO] "
    "EGO target waypoint=(%d,%d), "
    "ENU=(%.2f, %.2f, %.2f)",
    waypoint.ix,
    waypoint.iy,
    waypoint.x,
    waypoint.y,
    waypoint.z);
}


// ============================================================================
// Exit
// ============================================================================

void AlignDropSnakeEgoTask::onExit(
  Context &ctx,
  MavrosIface &iface) {

  if (phase_ == Phase::AVOIDING) {
    ego_.onExit(ctx, iface);
  }

  if (phase_ == Phase::ALIGNING) {
    align_.onExit(ctx, iface);
  }

  if (phase_ == Phase::ALIGNING_RED) {
    red_cross_.onExit(ctx, iface);
  }

  if (phase_ == Phase::DROPPING) {
    down_drop_.onExit(ctx, iface);
  }

  if (phase_ == Phase::LANDING) {
    land_.onExit(ctx, iface);
  }
}


// ============================================================================
// Contour freshness
// ============================================================================

bool AlignDropSnakeEgoTask::contourFresh(
  const Context &ctx) const {

  if (
    ctx.down_contour_seq == 0 ||
    ctx.down_contour_stamp ==
      std::chrono::steady_clock::time_point{}) {

    return false;
  }


  const auto now =
    std::chrono::steady_clock::now();


  return
    ctx.down_contour_stamp <= now &&
    now - ctx.down_contour_stamp <=
      std::chrono::duration<double>(
        std::max(
          0.0,
          cfg_.contour_timeout_s));
}


bool AlignDropSnakeEgoTask::redCrossFresh(
  const Context &ctx) const {

  if (ctx.red_cross_seq == 0 ||
      ctx.red_cross_stamp == std::chrono::steady_clock::time_point{}) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  return ctx.red_cross_stamp <= now &&
         now - ctx.red_cross_stamp <=
           std::chrono::duration<double>(
             std::max(0.0, cfg_.contour_timeout_s));
}


// ============================================================================
// Circle freshness
// ============================================================================

bool AlignDropSnakeEgoTask::circleFresh(
  const Context &ctx) const {

  if (
    ctx.down_circle_seq == 0 ||
    ctx.down_circle_stamp ==
      std::chrono::steady_clock::time_point{}) {

    return false;
  }


  const auto now =
    std::chrono::steady_clock::now();


  return
    ctx.down_circle_stamp <= now &&
    now - ctx.down_circle_stamp <=
      std::chrono::duration<double>(
        std::max(
          0.0,
          cfg_.contour_timeout_s));
}


// ============================================================================
// Circle position -> ENU
// ============================================================================

bool AlignDropSnakeEgoTask::circlePositionEnu(
  const Context &ctx,
  Vec3 &position) const {

  if (
    !circleFresh(ctx) ||
    !ctx.position_valid ||
    !ctx.finitePosition() ||
    !std::isfinite(ctx.yaw_enu) ||
    !std::isfinite(
      ctx.down_circle_offset_px.x) ||
    !std::isfinite(
      ctx.down_circle_offset_px.y) ||
    !std::isfinite(
      cfg_.align_pixels_per_meter) ||
    cfg_.align_pixels_per_meter <= 0.0) {

    return false;
  }


  const double forward =
    ctx.down_circle_offset_px.x /
    cfg_.align_pixels_per_meter;


  const double left =
    ctx.down_circle_offset_px.y /
    cfg_.align_pixels_per_meter;


  const double c =
    std::cos(ctx.yaw_enu);

  const double s =
    std::sin(ctx.yaw_enu);


  position = {

    ctx.position_enu.x +
      c * forward -
      s * left,

    ctx.position_enu.y +
      s * forward +
      c * left,

    ctx.position_enu.z
  };


  return true;
}


bool AlignDropSnakeEgoTask::contourPositionEnu(
  const Context &ctx,
  Vec3 &position) const {

  if (
    !contourFresh(ctx) ||
    !ctx.position_valid ||
    !ctx.finitePosition() ||
    !std::isfinite(ctx.yaw_enu) ||
    !std::isfinite(ctx.down_contour_offset_px.x) ||
    !std::isfinite(ctx.down_contour_offset_px.y) ||
    !std::isfinite(cfg_.align_pixels_per_meter) ||
    cfg_.align_pixels_per_meter <= 0.0) {

    return false;
  }

  const double forward =
    ctx.down_contour_offset_px.x / cfg_.align_pixels_per_meter;
  const double left =
    ctx.down_contour_offset_px.y / cfg_.align_pixels_per_meter;
  const double c = std::cos(ctx.yaw_enu);
  const double s = std::sin(ctx.yaw_enu);

  position = {
    ctx.position_enu.x + c * forward - s * left,
    ctx.position_enu.y + s * forward + c * left,
    ctx.position_enu.z
  };

  return true;
}


bool AlignDropSnakeEgoTask::redCrossPositionEnu(
  const Context &ctx,
  Vec3 &position) const {

  if (!redCrossFresh(ctx) ||
      !ctx.position_valid ||
      !ctx.finitePosition() ||
      !std::isfinite(ctx.yaw_enu) ||
      !std::isfinite(ctx.red_cross_offset_px.x) ||
      !std::isfinite(ctx.red_cross_offset_px.y) ||
      !std::isfinite(cfg_.red_cross_pixels_per_meter) ||
      cfg_.red_cross_pixels_per_meter <= 0.0) {
    return false;
  }

  const double forward =
    ctx.red_cross_offset_px.x / cfg_.red_cross_pixels_per_meter;
  const double left =
    ctx.red_cross_offset_px.y / cfg_.red_cross_pixels_per_meter;
  const double c = std::cos(ctx.yaw_enu);
  const double s = std::sin(ctx.yaw_enu);
  position = {
    ctx.position_enu.x + c * forward - s * left,
    ctx.position_enu.y + s * forward + c * left,
    ctx.position_enu.z
  };
  return true;
}


// ============================================================================
// Completed target suppression
// ============================================================================

bool AlignDropSnakeEgoTask::insideCompletedDownTargetRadius(
  const Context &ctx) const {

  std::array<Vec3, 2> current_targets{};
  std::array<const char *, 2> source_names{};
  std::size_t target_count = 0;

  if (circlePositionEnu(ctx, current_targets[target_count])) {
    source_names[target_count++] = "circle";
  }
  if (contourPositionEnu(ctx, current_targets[target_count])) {
    source_names[target_count++] = "contour";
  }

  if (target_count == 0) {

    return false;
  }

  double nearest_distance = std::numeric_limits<double>::infinity();
  const char *nearest_source = "unknown";

  for (std::size_t i = 0; i < target_count; ++i) {
    if (lost_target_cooldown_s_ > 0.0 &&
        std::hypot(current_targets[i].x - lost_target_.x,
                   current_targets[i].y - lost_target_.y) <= cfg_.align_retrigger_radius_m) {
      return true;
    }
    for (const auto &completed_target : completed_targets_enu_) {
      const double distance = std::hypot(
          current_targets[i].x - completed_target.x,
          current_targets[i].y - completed_target.y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_source = source_names[i];
      }
      if (distance <= cfg_.align_retrigger_radius_m) {
        RCLCPP_INFO_THROTTLE(
          logger_,
          *clock_,
          2000,
          "[ALIGN_DROP_SNAKE_EGO] skipping completed target: "
          "source=%s distance=%.3f m radius=%.3f m",
          source_names[i],
          distance,
          cfg_.align_retrigger_radius_m);
        return true;
      }
    }
  }

  if (!completed_targets_enu_.empty()) {
    RCLCPP_INFO_THROTTLE(
      logger_,
      *clock_,
      2000,
      "[ALIGN_DROP_SNAKE_EGO] new target candidate: "
      "source=%s nearest completed target=%.3f m radius=%.3f m",
      nearest_source,
      nearest_distance,
      cfg_.align_retrigger_radius_m);
  }

  return false;
}


// ============================================================================
// Occupancy grid freshness
// ============================================================================

bool AlignDropSnakeEgoTask::obstacleDataFresh(
  const Context &ctx) const {

  if (
    !ctx.occupancy_grid_valid ||
    ctx.occupancy_grid_resolution <= 0.0 ||
    ctx.occupancy_grid_width == 0 ||
    ctx.occupancy_grid_height == 0) {

    return false;
  }


  const auto now_us =
    nowUs(clock_);


  if (
    ctx.occupancy_grid_stamp_us == 0 ||
    now_us <
      ctx.occupancy_grid_stamp_us) {

    return false;
  }


  return
    now_us -
      ctx.occupancy_grid_stamp_us <=
    static_cast<std::uint64_t>(

      std::max(
        0.0,
        cfg_.occupancy_timeout_s) *
      1e6);
}


// ============================================================================
// Occupancy query
// ============================================================================

bool AlignDropSnakeEgoTask::occupiedAt(
  const Context &ctx,
  double x,
  double y) const {

  if (!obstacleDataFresh(ctx)) {
    return false;
  }


  const int ix =
    static_cast<int>(
      std::floor(
        (x -
          ctx.occupancy_grid_origin_x) /
        ctx.occupancy_grid_resolution));


  const int iy =
    static_cast<int>(
      std::floor(
        (y -
          ctx.occupancy_grid_origin_y) /
        ctx.occupancy_grid_resolution));


  const int cell_radius = static_cast<int>(std::ceil(
    std::max(0.0, cfg_.target_clearance_m) /
    ctx.occupancy_grid_resolution));
  for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      const double cell_distance = std::hypot(
        dx * ctx.occupancy_grid_resolution,
        dy * ctx.occupancy_grid_resolution);
      if (cell_distance > cfg_.target_clearance_m +
          std::sqrt(2.0) * ctx.occupancy_grid_resolution) {
        continue;
      }
      const int cell_x = ix + dx;
      const int cell_y = iy + dy;
      if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(ctx.occupancy_grid_width) ||
          cell_y >= static_cast<int>(ctx.occupancy_grid_height)) {
        return true;
      }
      const auto index = static_cast<std::size_t>(cell_y) *
        ctx.occupancy_grid_width + static_cast<std::size_t>(cell_x);
      if (index >= ctx.occupancy_grid_data.size() ||
          ctx.occupancy_grid_data[index] >= cfg_.occupied_threshold) {
        return true;
      }
    }
  }
  return false;
}


// ============================================================================
// Obstacle near segment
// ============================================================================

bool AlignDropSnakeEgoTask::obstacleNearSegment(
  const Context &ctx,
  double x0,
  double y0,
  double x1,
  double y1,
  Vec3 *nearest_obstacle) const {

  if (!obstacleDataFresh(ctx)) {
    return false;
  }


  const double resolution =
    ctx.occupancy_grid_resolution;


  const double speed = std::hypot(
    ctx.velocity_enu.x, ctx.velocity_enu.y);
  const double finite_speed = std::isfinite(speed) ? speed : 0.0;
  const double deceleration = std::max(
    1e-3, cfg_.braking_deceleration_mps2);
  const double braking_distance =
    finite_speed * std::max(0.0, cfg_.braking_control_delay_s) +
    finite_speed * finite_speed / (2.0 * deceleration) +
    std::max(0.0, cfg_.target_clearance_m) +
    std::sqrt(2.0) * resolution +
    std::max(0.0, cfg_.trigger_distance_m);
  const double corridor_margin =
    std::max(0.0, cfg_.target_clearance_m) +
    std::sqrt(2.0) * resolution;


  const int ix0 =
    static_cast<int>(
      std::floor(
        (
          std::min(x0, x1) -
          corridor_margin -
          ctx.occupancy_grid_origin_x
        ) /
        resolution));


  const int ix1 =
    static_cast<int>(
      std::floor(
        (
          std::max(x0, x1) +
          corridor_margin -
          ctx.occupancy_grid_origin_x
        ) /
        resolution));


  const int iy0 =
    static_cast<int>(
      std::floor(
        (
          std::min(y0, y1) -
          corridor_margin -
          ctx.occupancy_grid_origin_y
        ) /
        resolution));


  const int iy1 =
    static_cast<int>(
      std::floor(
        (
          std::max(y0, y1) +
          corridor_margin -
          ctx.occupancy_grid_origin_y
        ) /
        resolution));


  double nearest_distance = std::numeric_limits<double>::infinity();
  Vec3 nearest_position;
  for (
    int iy =
      std::max(0, iy0);

    iy <=
      std::min(
        iy1,
        static_cast<int>(
          ctx.occupancy_grid_height) - 1);

    ++iy) {


    for (
      int ix =
        std::max(0, ix0);

      ix <=
        std::min(
          ix1,
          static_cast<int>(
            ctx.occupancy_grid_width) - 1);

      ++ix) {


      const auto index =
        static_cast<std::size_t>(iy) *
          ctx.occupancy_grid_width +
        static_cast<std::size_t>(ix);


      if (
        index >=
          ctx.occupancy_grid_data.size() ||
        ctx.occupancy_grid_data[index] <
          cfg_.occupied_threshold) {

        continue;
      }


      const double cx =
        ctx.occupancy_grid_origin_x +
        (ix + 0.5) *
          resolution;


      const double cy =
        ctx.occupancy_grid_origin_y +
        (iy + 0.5) *
          resolution;


      const double distance_from_start = std::hypot(cx - x0, cy - y0);
      if (
        pointSegmentDistance(
          cx,
          cy,
          x0,
          y0,
          x1,
          y1) <= corridor_margin &&
        distance_from_start <= braking_distance &&
        distance_from_start < nearest_distance) {
        nearest_distance = distance_from_start;
        nearest_position = {cx, cy, ctx.position_enu.z};
      }
    }
  }


  if (!std::isfinite(nearest_distance)) {
    return false;
  }
  if (nearest_obstacle) {
    *nearest_obstacle = nearest_position;
  }
  return true;
}


// ============================================================================
// Select EGO avoidance target
// ============================================================================

bool AlignDropSnakeEgoTask::selectAvoidanceTarget(
  const Context &ctx) {

  SnakeGridTask::WaypointInfo waypoint;


  for (
    std::size_t i =
      snake_.currentIndex();

    snake_.waypointAt(
      i,
      waypoint);

    ++i) {


    if (
      !occupiedAt(
        ctx,
        waypoint.x,
        waypoint.y)) {

      avoidance_target_index_ = i;

      return true;
    }
  }


  return false;
}


// ============================================================================
// Landing
// ============================================================================

ITask::Status AlignDropSnakeEgoTask::tickLanding(
  Context &ctx,
  MavrosIface &iface,
  double dt_s) {

  const auto status =
    land_.tick(
      ctx,
      iface,
      dt_s);


  if (status ==
      Status::SUCCESS) {

    ctx.fault =
      failure_reason_;

    phase_ =
      Phase::FAILED;


    RCLCPP_INFO(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] "
      "landing completed after task failure: %s",
      failure_reason_.c_str());


    return Status::FAILURE;
  }


  if (status ==
      Status::FAILURE) {

    ctx.fault =
      failure_reason_ +
      "; landing failed";

    phase_ =
      Phase::FAILED;


    RCLCPP_ERROR(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] %s",
      ctx.fault.c_str());


    return Status::FAILURE;
  }


  return Status::RUNNING;
}


// ============================================================================
// Failure -> landing
// ============================================================================

void AlignDropSnakeEgoTask::failAndLand(
  Context &ctx,
  MavrosIface &iface,
  const char *reason) {

  if (
    phase_ == Phase::LANDING ||
    phase_ == Phase::FAILED) {

    return;
  }


  failure_reason_ =
    reason;


  if (phase_ ==
      Phase::AVOIDING) {

    ego_.onExit(
      ctx,
      iface);
  }


  land_.onEnter(
    ctx,
    iface);


  phase_ =
    Phase::LANDING;


  RCLCPP_ERROR(
    logger_,
    "[ALIGN_DROP_SNAKE_EGO] "
    "%s; starting landing",
    reason);
}

}  // namespace offboard_core_pkg
