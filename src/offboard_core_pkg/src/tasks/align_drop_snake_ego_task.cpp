#include "offboard_core_pkg/tasks/align_drop_snake_ego_task.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>

#include "offboard_core_pkg/mavros_iface.hpp"

namespace offboard_core_pkg {
namespace {

std::uint64_t nowUs(const rclcpp::Clock::SharedPtr &clock) {
  return static_cast<std::uint64_t>(clock->now().nanoseconds() / 1000ULL);
}

double pointSegmentDistance(
  double px, double py, double x0, double y0, double x1, double y1) {
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq < 1e-9) return std::hypot(px - x0, py - y0);
  const double t = std::clamp(
    ((px - x0) * dx + (py - y0) * dy) / length_sq, 0.0, 1.0);
  return std::hypot(px - (x0 + t * dx), py - (y0 + t * dy));
}

}  // namespace

AlignDropSnakeEgoTask::AlignDropSnakeEgoTask(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
  const Config &cfg)
: logger_(logger), clock_(std::move(clock)), cfg_(cfg), snake_(logger, cfg.snake),
  ego_(logger, clock_, std::move(goal_pub), cfg.ego),
  align_(cfg.align_pixels_per_meter, cfg.align_stable_frames,
         cfg.align_arrive_distance_m, cfg.align_max_step_m),
  land_(cfg.landing_timeout_s, cfg.landing_retry_interval_s) {}

std::string AlignDropSnakeEgoTask::name() const { return "ALIGN_DROP_SNAKE_EGO"; }

const char *AlignDropSnakeEgoTask::phaseName() const {
  switch (phase_) {
    case Phase::SNAKE: return "SNAKE";
    case Phase::AVOIDING: return "AVOIDING";
    case Phase::ALIGNING: return "ALIGNING";
    case Phase::RETURNING: return "RETURNING";
    case Phase::LANDING: return "LANDING";
    case Phase::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

void AlignDropSnakeEgoTask::onEnter(Context &ctx, MavrosIface &iface) {
  phase_ = Phase::SNAKE;
  avoidance_target_index_ = 0;
  avoidance_elapsed_s_ = 0.0;
  align_elapsed_s_ = 0.0;
  failure_reason_.clear();
  alignment_latched_ = false;
  completed_target_valid_ = false;
  snake_.onEnter(ctx, iface);
  if (snake_.failed()) {
    phase_ = Phase::FAILED;
    ctx.fault = "snake route initialization failed";
    RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] %s", ctx.fault.c_str());
    return;
  }
  RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] started");
}

ITask::Status AlignDropSnakeEgoTask::tick(
  Context &ctx, MavrosIface &iface, double dt_s) {
  const double dt = std::clamp(std::isfinite(dt_s) ? dt_s : 0.0, 0.0, 0.2);
  if (phase_ == Phase::FAILED) return Status::FAILURE;
  if (phase_ == Phase::LANDING) return tickLanding(ctx, iface, dt);
  if (phase_ == Phase::ALIGNING) return tickAlignment(ctx, iface, dt);
  if (phase_ == Phase::RETURNING) return tickReturn(ctx, iface, dt);

  if (contourFresh(ctx) && !alignment_latched_ &&
      !insideCompletedTargetRadius(ctx)) {
    beginAlignment(ctx, iface, phase_ == Phase::AVOIDING ?
      ResumePhase::AVOIDING : ResumePhase::SNAKE);
    return Status::RUNNING;
  }

  if (phase_ == Phase::SNAKE) {
    if (snake_.finished()) {
      RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] snake route completed");
      return Status::SUCCESS;
    }

    SnakeGridTask::WaypointInfo waypoint;
    while (snake_.waypointAt(snake_.currentIndex(), waypoint) &&
           occupiedAt(ctx, waypoint.x, waypoint.y)) {
      RCLCPP_WARN(logger_, "[ALIGN_DROP_SNAKE_EGO] skip occupied cell=(%d,%d)",
                  waypoint.ix, waypoint.iy);
      snake_.skipCurrentWaypoint();
    }
    if (snake_.finished()) {
      RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] snake route completed after occupied cells were skipped");
      return Status::SUCCESS;
    }
    if (!snake_.waypointAt(snake_.currentIndex(), waypoint)) {
      ctx.fault = "snake waypoint index is invalid";
      RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] %s at index=%zu",
                   ctx.fault.c_str(), snake_.currentIndex());
      return Status::FAILURE;
    }

    if (obstacleDataFresh(ctx) && obstacleNearSegment(
        ctx, ctx.position_enu.x, ctx.position_enu.y, waypoint.x, waypoint.y)) {
      if (!selectAvoidanceTarget(ctx)) {
        RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] obstacle near waypoint=(%d,%d), no unoccupied avoidance target",
                     waypoint.ix, waypoint.iy);
        failAndLand(ctx, iface, "no safe snake waypoint for EGO avoidance");
        return Status::RUNNING;
      }
      RCLCPP_WARN(logger_, "[ALIGN_DROP_SNAKE_EGO] obstacle detected; diverting from waypoint=(%d,%d) to index=%zu",
                  waypoint.ix, waypoint.iy, avoidance_target_index_);
      beginAvoidance(ctx, iface);
      phase_ = Phase::AVOIDING;
      return Status::RUNNING;
    }

    const auto status = snake_.tick(ctx, iface, dt);
    if (status == Status::FAILURE) {
      failAndLand(ctx, iface, "snake task failed");
      return Status::RUNNING;
    }
    return status;
  }

  avoidance_elapsed_s_ += dt;
  if (avoidance_elapsed_s_ >= cfg_.avoidance_timeout_s) {
    RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] avoidance timeout after %.1f s (limit %.1f s)",
                 avoidance_elapsed_s_, cfg_.avoidance_timeout_s);
    failAndLand(ctx, iface, "EGO avoidance timed out");
    return Status::RUNNING;
  }
  const auto status = ego_.tick(ctx, iface, dt);
  if (status == Status::SUCCESS) {
    RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] EGO avoidance reached index=%zu; resuming snake route",
                avoidance_target_index_);
    ego_.onExit(ctx, iface);
    snake_.resumeAfterWaypoint(avoidance_target_index_, ctx);
    phase_ = Phase::SNAKE;
    avoidance_elapsed_s_ = 0.0;
  } else if (status == Status::FAILURE) {
    RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] EGO avoidance task reported failure");
    failAndLand(ctx, iface, "EGO avoidance failed");
  }
  return Status::RUNNING;
}

void AlignDropSnakeEgoTask::beginAlignment(
  Context &ctx, MavrosIface &iface, ResumePhase resume_phase) {
  resume_phase_ = resume_phase;
  resume_position_ = ctx.position_enu;
  resume_yaw_ = ctx.yaw_enu;
  if (resume_phase_ == ResumePhase::SNAKE) {
    snake_.onPause(ctx, iface);
  } else {
    ego_.onExit(ctx, iface);
  }
  align_.onEnter(ctx, iface);
  align_elapsed_s_ = 0.0;
  alignment_latched_ = true;
  phase_ = Phase::ALIGNING;
  RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] alignment triggered in %s",
              resume_phase_ == ResumePhase::AVOIDING ? "EGO" : "snake");
}

ITask::Status AlignDropSnakeEgoTask::tickAlignment(
  Context &ctx, MavrosIface &iface, double dt_s) {
  align_elapsed_s_ += dt_s;
  if (align_elapsed_s_ >= cfg_.align_timeout_s) {
    align_.onExit(ctx, iface);
    ctx.fault = "down alignment timed out";
    phase_ = Phase::FAILED;
    RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] %s after %.1f s",
                 ctx.fault.c_str(), align_elapsed_s_);
    return Status::FAILURE;
  }
  const auto status = align_.tick(ctx, iface, dt_s);
  if (status == Status::FAILURE) {
    align_.onExit(ctx, iface);
    ctx.fault = "down alignment failed";
    phase_ = Phase::FAILED;
    RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] %s", ctx.fault.c_str());
    return Status::FAILURE;
  }
  if (status == Status::SUCCESS) {
    Vec3 target;
    if (circlePositionEnu(ctx, target)) {
      completed_target_enu_ = target;
      completed_target_valid_ = true;
      RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] alignment succeeded; target ENU=(%.2f, %.2f, %.2f)",
                  target.x, target.y, target.z);
    } else {
      RCLCPP_WARN(logger_, "[ALIGN_DROP_SNAKE_EGO] alignment succeeded, but circle target is unavailable; retrigger suppression disabled");
    }
    alignment_latched_ = false;
    align_.onExit(ctx, iface);
    ctx.position_setpoint_enu = resume_position_;
    ctx.yaw_setpoint_enu = resume_yaw_;
    phase_ = Phase::RETURNING;
    RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] returning to saved position=(%.2f, %.2f, %.2f)",
                resume_position_.x, resume_position_.y, resume_position_.z);
  }
  return Status::RUNNING;
}

ITask::Status AlignDropSnakeEgoTask::tickReturn(
  Context &ctx, MavrosIface &iface, double dt_s) {
  if (!ctx.position_valid || !ctx.finitePosition()) return Status::RUNNING;
  const double dx = resume_position_.x - ctx.position_enu.x;
  const double dy = resume_position_.y - ctx.position_enu.y;
  const double dz = resume_position_.z - ctx.position_enu.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double tolerance = std::min(cfg_.snake.arrive_xy_m, cfg_.snake.arrive_z_m);
  const double max_step = std::max(0.01, cfg_.snake.max_step_m) *
    std::clamp(dt_s, 0.02, 0.1) / 0.05;
  if (distance <= tolerance) {
    ctx.position_setpoint_enu = resume_position_;
    ctx.yaw_setpoint_enu = resume_yaw_;
    if (resume_phase_ == ResumePhase::AVOIDING) {
      ego_.onEnter(ctx, iface);
    }
    phase_ = resume_phase_ == ResumePhase::AVOIDING ? Phase::AVOIDING : Phase::SNAKE;
    RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] returned to saved position (distance %.2f m); resuming %s",
                distance, phase_ == Phase::AVOIDING ? "EGO avoidance" : "snake route");
    return Status::RUNNING;
  }
  const double scale = std::min(1.0, max_step / distance);
  ctx.position_setpoint_enu = {
    ctx.position_enu.x + dx * scale,
    ctx.position_enu.y + dy * scale,
    ctx.position_enu.z + dz * scale};
  ctx.yaw_setpoint_enu = resume_yaw_;
  ctx.publish_position_setpoint = true;
  ctx.setpoint_mode = SetpointMode::POSITION;
  ctx.use_position_velocity_acceleration = false;
  return Status::RUNNING;
}

void AlignDropSnakeEgoTask::beginAvoidance(Context &ctx, MavrosIface &iface) {
  SnakeGridTask::WaypointInfo waypoint;
  snake_.waypointAt(avoidance_target_index_, waypoint);
  snake_.onPause(ctx, iface);
  ego_.setTargetEnu(waypoint.x, waypoint.y, waypoint.z);
  ego_.onEnter(ctx, iface);
  avoidance_elapsed_s_ = 0.0;
  RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] EGO target waypoint=(%d,%d), ENU=(%.2f, %.2f, %.2f)",
              waypoint.ix, waypoint.iy, waypoint.x, waypoint.y, waypoint.z);
}

void AlignDropSnakeEgoTask::onExit(Context &ctx, MavrosIface &iface) {
  if (phase_ == Phase::AVOIDING) ego_.onExit(ctx, iface);
  if (phase_ == Phase::ALIGNING) align_.onExit(ctx, iface);
  if (phase_ == Phase::LANDING) land_.onExit(ctx, iface);
}

bool AlignDropSnakeEgoTask::contourFresh(const Context &ctx) const {
  if (ctx.down_contour_seq == 0 ||
      ctx.down_contour_stamp == std::chrono::steady_clock::time_point{}) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  return ctx.down_contour_stamp <= now &&
    now - ctx.down_contour_stamp <=
      std::chrono::duration<double>(std::max(0.0, cfg_.contour_timeout_s));
}

bool AlignDropSnakeEgoTask::circleFresh(const Context &ctx) const {
  if (ctx.down_circle_seq == 0 ||
      ctx.down_circle_stamp == std::chrono::steady_clock::time_point{}) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  return ctx.down_circle_stamp <= now &&
    now - ctx.down_circle_stamp <=
      std::chrono::duration<double>(std::max(0.0, cfg_.contour_timeout_s));
}

bool AlignDropSnakeEgoTask::circlePositionEnu(
  const Context &ctx, Vec3 &position) const {
  if (!circleFresh(ctx) || !ctx.position_valid || !ctx.finitePosition() ||
      !std::isfinite(ctx.yaw_enu) ||
      !std::isfinite(ctx.down_circle_offset_px.x) ||
      !std::isfinite(ctx.down_circle_offset_px.y) ||
      !std::isfinite(cfg_.align_pixels_per_meter) ||
      cfg_.align_pixels_per_meter <= 0.0) {
    return false;
  }
  const double forward = ctx.down_circle_offset_px.x / cfg_.align_pixels_per_meter;
  const double left = ctx.down_circle_offset_px.y / cfg_.align_pixels_per_meter;
  const double c = std::cos(ctx.yaw_enu);
  const double s = std::sin(ctx.yaw_enu);
  position = {
    ctx.position_enu.x + c * forward - s * left,
    ctx.position_enu.y + s * forward + c * left,
    ctx.position_enu.z};
  return true;
}

bool AlignDropSnakeEgoTask::insideCompletedTargetRadius(const Context &ctx) const {
  Vec3 current_target;
  if (!completed_target_valid_ || !circlePositionEnu(ctx, current_target)) return false;
  return std::hypot(
    current_target.x - completed_target_enu_.x,
    current_target.y - completed_target_enu_.y) <= cfg_.align_retrigger_radius_m;
}

bool AlignDropSnakeEgoTask::obstacleDataFresh(const Context &ctx) const {
  if (!ctx.occupancy_grid_valid || ctx.occupancy_grid_resolution <= 0.0 ||
      ctx.occupancy_grid_width == 0 || ctx.occupancy_grid_height == 0) return false;
  const auto now_us = nowUs(clock_);
  if (ctx.occupancy_grid_stamp_us == 0 || now_us < ctx.occupancy_grid_stamp_us) return false;
  return now_us - ctx.occupancy_grid_stamp_us <= static_cast<std::uint64_t>(
    std::max(0.0, cfg_.occupancy_timeout_s) * 1e6);
}

bool AlignDropSnakeEgoTask::occupiedAt(
  const Context &ctx, double x, double y) const {
  if (!obstacleDataFresh(ctx)) return false;
  const int ix = static_cast<int>(std::floor(
    (x - ctx.occupancy_grid_origin_x) / ctx.occupancy_grid_resolution));
  const int iy = static_cast<int>(std::floor(
    (y - ctx.occupancy_grid_origin_y) / ctx.occupancy_grid_resolution));
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(ctx.occupancy_grid_width) ||
      iy >= static_cast<int>(ctx.occupancy_grid_height)) return false;
  const auto index = static_cast<std::size_t>(iy) * ctx.occupancy_grid_width +
    static_cast<std::size_t>(ix);
  return index < ctx.occupancy_grid_data.size() &&
    ctx.occupancy_grid_data[index] >= cfg_.occupied_threshold;
}

bool AlignDropSnakeEgoTask::obstacleNearSegment(
  const Context &ctx, double x0, double y0, double x1, double y1) const {
  if (!obstacleDataFresh(ctx)) return false;
  const double resolution = ctx.occupancy_grid_resolution;
  const double margin = std::max(0.0, cfg_.trigger_distance_m) +
    std::sqrt(2.0) * resolution;
  const int ix0 = static_cast<int>(std::floor(
    (std::min(x0, x1) - margin - ctx.occupancy_grid_origin_x) / resolution));
  const int ix1 = static_cast<int>(std::floor(
    (std::max(x0, x1) + margin - ctx.occupancy_grid_origin_x) / resolution));
  const int iy0 = static_cast<int>(std::floor(
    (std::min(y0, y1) - margin - ctx.occupancy_grid_origin_y) / resolution));
  const int iy1 = static_cast<int>(std::floor(
    (std::max(y0, y1) + margin - ctx.occupancy_grid_origin_y) / resolution));
  for (int iy = std::max(0, iy0);
       iy <= std::min(iy1, static_cast<int>(ctx.occupancy_grid_height) - 1); ++iy) {
    for (int ix = std::max(0, ix0);
         ix <= std::min(ix1, static_cast<int>(ctx.occupancy_grid_width) - 1); ++ix) {
      const auto index = static_cast<std::size_t>(iy) * ctx.occupancy_grid_width +
        static_cast<std::size_t>(ix);
      if (index >= ctx.occupancy_grid_data.size() ||
          ctx.occupancy_grid_data[index] < cfg_.occupied_threshold) continue;
      const double cx = ctx.occupancy_grid_origin_x + (ix + 0.5) * resolution;
      const double cy = ctx.occupancy_grid_origin_y + (iy + 0.5) * resolution;
      if (pointSegmentDistance(cx, cy, x0, y0, x1, y1) <= margin) return true;
    }
  }
  return false;
}

bool AlignDropSnakeEgoTask::selectAvoidanceTarget(const Context &ctx) {
  SnakeGridTask::WaypointInfo waypoint;
  for (std::size_t i = snake_.currentIndex(); snake_.waypointAt(i, waypoint); ++i) {
    if (!occupiedAt(ctx, waypoint.x, waypoint.y)) {
      avoidance_target_index_ = i;
      return true;
    }
  }
  return false;
}

ITask::Status AlignDropSnakeEgoTask::tickLanding(
  Context &ctx, MavrosIface &iface, double dt_s) {
  const auto status = land_.tick(ctx, iface, dt_s);
  if (status == Status::SUCCESS) {
    ctx.fault = failure_reason_;
    phase_ = Phase::FAILED;
    RCLCPP_INFO(logger_, "[ALIGN_DROP_SNAKE_EGO] landing completed after task failure: %s",
                failure_reason_.c_str());
    return Status::FAILURE;
  }
  if (status == Status::FAILURE) {
    ctx.fault = failure_reason_ + "; landing failed";
    phase_ = Phase::FAILED;
    RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] %s", ctx.fault.c_str());
    return Status::FAILURE;
  }
  return Status::RUNNING;
}

void AlignDropSnakeEgoTask::failAndLand(
  Context &ctx, MavrosIface &iface, const char *reason) {
  if (phase_ == Phase::LANDING || phase_ == Phase::FAILED) return;
  failure_reason_ = reason;
  if (phase_ == Phase::AVOIDING) ego_.onExit(ctx, iface);
  land_.onEnter(ctx, iface);
  phase_ = Phase::LANDING;
  RCLCPP_ERROR(logger_, "[ALIGN_DROP_SNAKE_EGO] %s; starting landing", reason);
}

}  // namespace offboard_core_pkg
