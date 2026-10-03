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

    case Phase::AVOIDING:
      return "AVOIDING";

    case Phase::ALIGNING:
      return "ALIGNING";

    case Phase::RETURNING:
      return "RETURNING";

    case Phase::LANDING:
      return "LANDING";

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
  align_elapsed_s_ = 0.0;

  failure_reason_.clear();

  alignment_latched_ = false;
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

  // --------------------------------------------------------------------------
  // 独立状态
  // --------------------------------------------------------------------------

  if (phase_ == Phase::FAILED) {
    return Status::FAILURE;
  }

  if (phase_ == Phase::LANDING) {
    return tickLanding(ctx, iface, dt);
  }

  if (phase_ == Phase::ALIGNING) {
    return tickAlignment(ctx, iface, dt);
  }

  if (phase_ == Phase::RETURNING) {
    return tickReturn(ctx, iface, dt);
  }


  // --------------------------------------------------------------------------
  // 检测到轮廓 -> 触发下视纠偏
  // --------------------------------------------------------------------------

  if (contourFresh(ctx) &&
      !alignment_latched_ &&
      !insideCompletedTargetRadius(ctx)) {

    beginAlignment(
      ctx,
      iface,
      phase_ == Phase::AVOIDING
        ? ResumePhase::AVOIDING
        : ResumePhase::SNAKE);

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
        waypoint.y)) {

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

      phase_ = Phase::AVOIDING;

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
  ResumePhase resume_phase) {

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


  align_.onEnter(ctx, iface);

  alignment_target_valid_ = false;

  align_elapsed_s_ = 0.0;

  alignment_latched_ = true;

  phase_ = Phase::ALIGNING;


  RCLCPP_INFO(
    logger_,
    "[ALIGN_DROP_SNAKE_EGO] alignment triggered in %s",
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

  align_elapsed_s_ += dt_s;


  // --------------------------------------------------------------------------
  // 对准超时
  // --------------------------------------------------------------------------

  if (align_elapsed_s_ >=
      cfg_.align_timeout_s) {

    align_.onExit(ctx, iface);

    // Abandon this attempt and return to the saved entry pose. Suppress this
    // target after returning so a still-fresh detection cannot retrigger it.
    Vec3 target;
    if (circlePositionEnu(ctx, target) ||
        contourPositionEnu(ctx, target)) {
      completed_targets_enu_.push_back(target);
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

    RCLCPP_WARN(
      logger_,
      "[ALIGN_DROP_SNAKE_EGO] down alignment timed out after %.1f s; "
      "returning to the alignment entry pose",
      align_elapsed_s_);

    return Status::RUNNING;
  }


  // --------------------------------------------------------------------------
  // 对准任务
  // --------------------------------------------------------------------------

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

    align_.onExit(ctx, iface);


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


// ============================================================================
// Completed target suppression
// ============================================================================

bool AlignDropSnakeEgoTask::insideCompletedTargetRadius(
  const Context &ctx) const {

  Vec3 current_target;

  if (!contourPositionEnu(ctx, current_target)) {

    return false;
  }

  double nearest_distance = std::numeric_limits<double>::infinity();

  for (const auto &completed_target : completed_targets_enu_) {
    const double distance = std::hypot(
        current_target.x - completed_target.x,
        current_target.y - completed_target.y);
    nearest_distance = std::min(nearest_distance, distance);
    if (distance <= cfg_.align_retrigger_radius_m) {
      RCLCPP_INFO_THROTTLE(
        logger_,
        *clock_,
        2000,
        "[ALIGN_DROP_SNAKE_EGO] skipping completed target: "
        "distance=%.2f m radius=%.2f m",
        distance,
        cfg_.align_retrigger_radius_m);
      return true;
    }
  }

  if (!completed_targets_enu_.empty()) {
    RCLCPP_INFO_THROTTLE(
      logger_,
      *clock_,
      2000,
      "[ALIGN_DROP_SNAKE_EGO] new contour candidate: "
      "nearest completed target=%.2f m radius=%.2f m",
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


  if (
    ix < 0 ||
    iy < 0 ||
    ix >=
      static_cast<int>(
        ctx.occupancy_grid_width) ||
    iy >=
      static_cast<int>(
        ctx.occupancy_grid_height)) {

    return false;
  }


  const auto index =
    static_cast<std::size_t>(iy) *
      ctx.occupancy_grid_width +
    static_cast<std::size_t>(ix);


  return
    index <
      ctx.occupancy_grid_data.size() &&
    ctx.occupancy_grid_data[index] >=
      cfg_.occupied_threshold;
}


// ============================================================================
// Obstacle near segment
// ============================================================================

bool AlignDropSnakeEgoTask::obstacleNearSegment(
  const Context &ctx,
  double x0,
  double y0,
  double x1,
  double y1) const {

  if (!obstacleDataFresh(ctx)) {
    return false;
  }


  const double resolution =
    ctx.occupancy_grid_resolution;


  const double margin =
    std::max(
      0.0,
      cfg_.trigger_distance_m) +
    std::sqrt(2.0) *
      resolution;


  const int ix0 =
    static_cast<int>(
      std::floor(
        (
          std::min(x0, x1) -
          margin -
          ctx.occupancy_grid_origin_x
        ) /
        resolution));


  const int ix1 =
    static_cast<int>(
      std::floor(
        (
          std::max(x0, x1) +
          margin -
          ctx.occupancy_grid_origin_x
        ) /
        resolution));


  const int iy0 =
    static_cast<int>(
      std::floor(
        (
          std::min(y0, y1) -
          margin -
          ctx.occupancy_grid_origin_y
        ) /
        resolution));


  const int iy1 =
    static_cast<int>(
      std::floor(
        (
          std::max(y0, y1) +
          margin -
          ctx.occupancy_grid_origin_y
        ) /
        resolution));


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


      if (
        pointSegmentDistance(
          cx,
          cy,
          x0,
          y0,
          x1,
          y1) <= margin) {

        return true;
      }
    }
  }


  return false;
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
