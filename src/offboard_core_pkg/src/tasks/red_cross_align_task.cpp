#include "offboard_core_pkg/tasks/red_cross_align_task.hpp"

#include <algorithm>
#include <cmath>

#include "offboard_core_pkg/context.hpp"
#include "rclcpp/rclcpp.hpp"

namespace offboard_core_pkg {
namespace {

constexpr auto kDetectionTimeout = std::chrono::milliseconds(500);

rclcpp::Logger logger() {
  return rclcpp::get_logger("red_cross_align");
}

bool fresh(std::chrono::steady_clock::time_point stamp) {
  const auto now = std::chrono::steady_clock::now();
  return stamp != std::chrono::steady_clock::time_point{} &&
      stamp <= now && now - stamp <= kDetectionTimeout;
}

}  // namespace

RedCrossAlignTask::RedCrossAlignTask(
    double pixels_per_meter,
    int stable_frames,
    double arrive_distance_m,
    double max_step_m)
    : pixels_per_meter_(pixels_per_meter),
      stable_frames_(stable_frames),
      arrive_distance_m_(arrive_distance_m),
      max_step_m_(max_step_m),
      valid_config_(std::isfinite(pixels_per_meter) && pixels_per_meter > 0.0 &&
                    stable_frames > 0 && std::isfinite(arrive_distance_m) &&
                    arrive_distance_m > 0.0 && std::isfinite(max_step_m) &&
                    max_step_m > 0.0) {}

std::string RedCrossAlignTask::name() const {
  return "red_cross_align";
}

void RedCrossAlignTask::holdPosition(Context &ctx) const {
  if (!ctx.position_valid || !ctx.finitePosition()) return;
  ctx.position_setpoint_enu = {ctx.position_enu.x, ctx.position_enu.y, align_height_};
  if (std::isfinite(ctx.home_yaw_enu)) ctx.yaw_setpoint_enu = ctx.home_yaw_enu;
  ctx.setpoint_mode = SetpointMode::POSITION;
  ctx.use_position_velocity_acceleration = false;
  ctx.publish_position_setpoint = true;
}

void RedCrossAlignTask::onEnter(Context &ctx, MavrosIface &) {
  stable_count_ = 0;
  align_height_ = ctx.position_enu.z;
  last_seq_ = ctx.red_cross_seq;
  detection_was_available_ = false;
  last_status_log_ = {};
  ctx.fault.clear();

  if (!valid_config_) {
    ctx.fault = "red_cross_align parameters are invalid";
    RCLCPP_ERROR(logger(), "[RED_CROSS_ALIGN] invalid config");
    return;
  }
  if (!std::isfinite(ctx.home_yaw_enu)) {
    ctx.fault = "red_cross_align home yaw is invalid";
    RCLCPP_ERROR(logger(), "[RED_CROSS_ALIGN] home_yaw_enu is invalid");
    return;
  }
  holdPosition(ctx);
  RCLCPP_INFO(logger(),
      "[RED_CROSS_ALIGN] started: pixels_per_meter=%.2f stable_frames=%d "
      "arrive_distance=%.3f m max_step=%.3f m hold_height=%.2f m",
      pixels_per_meter_, stable_frames_, arrive_distance_m_, max_step_m_, align_height_);
}

ITask::Status RedCrossAlignTask::tick(Context &ctx, MavrosIface &, double) {
  if (!valid_config_) return Status::FAILURE;
  if (!ctx.connected || !ctx.position_valid || !ctx.finitePosition() ||
      !std::isfinite(ctx.yaw_enu) || !std::isfinite(ctx.home_yaw_enu)) {
    detection_was_available_ = false;
    stable_count_ = 0;
    return Status::RUNNING;
  }

  if (ctx.red_cross_seq == 0 || !fresh(ctx.red_cross_stamp)) {
    if (detection_was_available_) {
      RCLCPP_WARN(logger(), "[RED_CROSS_ALIGN] detection timed out; holding position");
    }
    detection_was_available_ = false;
    stable_count_ = 0;
    holdPosition(ctx);
    return Status::RUNNING;
  }
  if (!detection_was_available_) {
    RCLCPP_INFO(logger(), "[RED_CROSS_ALIGN] fresh detection available");
  }
  detection_was_available_ = true;
  if (ctx.red_cross_seq == last_seq_) return Status::RUNNING;
  last_seq_ = ctx.red_cross_seq;

  const auto &offset = ctx.red_cross_offset_px;
  if (!std::isfinite(offset.x) || !std::isfinite(offset.y)) {
    stable_count_ = 0;
    holdPosition(ctx);
    RCLCPP_WARN(logger(), "[RED_CROSS_ALIGN] received non-finite offset");
    return Status::RUNNING;
  }

  const double forward = offset.x / pixels_per_meter_;
  const double left = offset.y / pixels_per_meter_;
  const double distance = std::hypot(forward, left);
  if (distance <= arrive_distance_m_) {
    holdPosition(ctx);
    ++stable_count_;
    if (stable_count_ >= stable_frames_) {
      RCLCPP_INFO(logger(), "[RED_CROSS_ALIGN] alignment complete: distance=%.3f m", distance);
      return Status::SUCCESS;
    }
    return Status::RUNNING;
  }

  stable_count_ = 0;
  double adaptive_max_step = max_step_m_;
  if (distance <= 0.15) adaptive_max_step = 0.04;
  else if (distance <= 0.30) adaptive_max_step = 0.08;
  const double scale = std::min(1.0, adaptive_max_step / distance);
  const double step_forward = forward * scale;
  const double step_left = left * scale;
  const double c = std::cos(ctx.yaw_enu);
  const double s = std::sin(ctx.yaw_enu);
  ctx.position_setpoint_enu = {
      ctx.position_enu.x + c * step_forward - s * step_left,
      ctx.position_enu.y + s * step_forward + c * step_left,
      align_height_};
  ctx.yaw_setpoint_enu = ctx.home_yaw_enu;
  ctx.setpoint_mode = SetpointMode::POSITION;
  ctx.use_position_velocity_acceleration = false;
  ctx.publish_position_setpoint = true;

  const auto now = std::chrono::steady_clock::now();
  if (last_status_log_ == std::chrono::steady_clock::time_point{} ||
      now - last_status_log_ >= std::chrono::seconds(1)) {
    RCLCPP_INFO(logger(),
        "[RED_CROSS_ALIGN] correcting: offset=(%.1f, %.1f) px distance=%.3f m "
        "step=%.3f m target=(%.2f, %.2f, %.2f)",
        offset.x, offset.y, distance, std::hypot(step_forward, step_left),
        ctx.position_setpoint_enu.x, ctx.position_setpoint_enu.y, ctx.position_setpoint_enu.z);
    last_status_log_ = now;
  }
  return Status::RUNNING;
}

void RedCrossAlignTask::onPause(Context &ctx, MavrosIface &) {
  stable_count_ = 0;
  detection_was_available_ = false;
  holdPosition(ctx);
}

void RedCrossAlignTask::onResume(Context &ctx, MavrosIface &) {
  stable_count_ = 0;
  detection_was_available_ = false;
  last_seq_ = ctx.red_cross_seq;
  holdPosition(ctx);
}

}  // namespace offboard_core_pkg
