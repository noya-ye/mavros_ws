#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/tasks/align_drop_snake_ego_task.hpp"

namespace offboard_core_pkg {
class MavrosIface {};
}

using namespace offboard_core_pkg;

static void exerciseFlow(bool red_cross) {
  Context ctx;
  ctx.connected = true;
  ctx.position_valid = true;
  ctx.position_enu = {0.0, 0.0, 1.0};
  ctx.yaw_enu = 0.0;
  ctx.home_yaw_enu = 0.0;
  ctx.yolo_detections = {{1, 0.9F}};
  ctx.yolo_detections_stamp = std::chrono::steady_clock::now();
  ctx.yolo_detections_seq = 1;
  ctx.down_contour_offset_px = {0.0, 0.0, 0.0};
  ctx.down_contour_stamp = std::chrono::steady_clock::now();
  ctx.down_contour_seq = 1;
  ctx.down_circle_offset_px = {0.0, 0.0, 0.0};
  ctx.down_circle_stamp = ctx.down_contour_stamp;
  ctx.down_circle_seq = 1;
  ctx.red_cross_offset_px = {0.0, 0.0, 0.0};
  ctx.red_cross_stamp = std::chrono::steady_clock::now();
  ctx.red_cross_seq = 1;
  if (!red_cross) ctx.red_cross_seq = 0;

  AlignDropSnakeEgoTask::Config cfg;
  cfg.red_cross_enabled = true;
  cfg.align_pixels_per_meter = 1000.0;
  cfg.align_stable_frames = 1;
  cfg.align_arrive_distance_m = 0.05;
  cfg.align_timeout_s = 2.0;
  cfg.red_cross_pixels_per_meter = 10.0;
  cfg.red_cross_stable_frames = 1;
  cfg.red_cross_arrive_distance_m = 0.05;
  cfg.red_cross_max_step_m = 0.2;
  cfg.red_cross_timeout_s = 2.0;
  cfg.drop_height_m = 0.5;
  cfg.drop_serial_device = "/dev/null";

  MavrosIface iface;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  AlignDropSnakeEgoTask task(
    rclcpp::get_logger("align_drop_snake_ego_test"), clock, nullptr, cfg);
  task.onEnter(ctx, iface);

  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  if (red_cross) {
    ctx.red_cross_offset_px = {10.0, 0.0, 0.0};
    ctx.red_cross_stamp = std::chrono::steady_clock::now();
    ++ctx.red_cross_seq;
    assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
    assert(std::abs(ctx.position_setpoint_enu.x - 0.2) < 1e-9);

    ctx.red_cross_offset_px = {0.0, 0.0, 0.0};
    ctx.red_cross_stamp = std::chrono::steady_clock::now();
    ++ctx.red_cross_seq;
    assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  } else {
    ++ctx.down_circle_seq;
    ctx.down_circle_stamp = std::chrono::steady_clock::now();
    assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  }

  assert(std::abs(ctx.position_setpoint_enu.z - 0.5) < 1e-9);
  ctx.position_enu.z = 0.5;
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(std::abs(ctx.position_setpoint_enu.x) < 1e-9);
  assert(std::abs(ctx.position_setpoint_enu.y - 0.10) < 1e-9);
  ctx.position_enu.y = 0.10;
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(std::abs(ctx.position_setpoint_enu.z - 1.0) < 1e-9);

  ctx.position_enu.y = 0.0;
  ctx.position_enu.z = 1.0;
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);

  const auto stale = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  ctx.down_circle_stamp = ctx.down_contour_stamp = stale;
  ctx.red_cross_stamp = std::chrono::steady_clock::now();
  ctx.red_cross_seq = 100;
  ctx.red_cross_offset_px = {10.0, 0.0, 0.0};
  task.tick(ctx, iface, 0.05);
  ++ctx.red_cross_seq;
  task.tick(ctx, iface, 0.05);
  if (red_cross) {
    // Success suppresses red even far outside the old spatial radius.
    assert(ctx.position_setpoint_enu.x <= 0.0);
    ctx.red_cross_stamp = stale;
    ctx.down_circle_offset_px = {0.0, 0.0, 0.0};
    ctx.down_circle_stamp = std::chrono::steady_clock::now();
    ++ctx.down_circle_seq;
    task.tick(ctx, iface, 0.05);
    ++ctx.down_circle_seq;
    task.tick(ctx, iface, 0.05);
    // A red completion does not suppress a circle at the same location.
    assert(std::abs(ctx.position_setpoint_enu.z - 0.5) < 1e-9);
  } else {
    // A completed down target at the same location does not suppress red.
    assert(std::abs(ctx.position_setpoint_enu.x - 0.2) < 1e-9);
  }
}

static void exerciseDetectionLoss(bool red_cross, bool recover) {
  Context ctx;
  ctx.connected = true;
  ctx.position_valid = true;
  ctx.position_enu = {0.0, 0.0, 1.0};
  ctx.yaw_enu = ctx.home_yaw_enu = 0.0;
  ctx.down_contour_offset_px = {100.0, 0.0, 0.0};
  ctx.red_cross_offset_px = {100.0, 0.0, 0.0};
  ctx.down_contour_stamp = ctx.red_cross_stamp = std::chrono::steady_clock::now();
  ctx.down_contour_seq = ctx.red_cross_seq = 1;
  AlignDropSnakeEgoTask::Config cfg;
  cfg.red_cross_enabled = red_cross;
  cfg.align_loss_timeout_s = 0.3;
  cfg.align_timeout_s = cfg.red_cross_timeout_s = 15.0;
  cfg.align_retrigger_radius_m = 0.9;
  MavrosIface iface;
  AlignDropSnakeEgoTask task(rclcpp::get_logger("loss_test"),
    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), nullptr, cfg);
  task.onEnter(ctx, iface);
  task.tick(ctx, iface, 0.05);
  ctx.position_enu.x = 0.2;
  const auto stale = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  ctx.down_contour_stamp = ctx.red_cross_stamp = stale;
  // A fresh detection from the other task must not keep this alignment alive.
  if (red_cross) ctx.down_contour_stamp = std::chrono::steady_clock::now();
  else ctx.red_cross_stamp = std::chrono::steady_clock::now();
  task.tick(ctx, iface, 0.2);
  assert(std::abs(ctx.position_setpoint_enu.x - 0.2) < 1e-9);
  if (recover) {
    // Recovery resets the continuous-loss interval. Down alignment also accepts circle fallback.
    if (red_cross) {
      ctx.red_cross_stamp = std::chrono::steady_clock::now();
      ++ctx.red_cross_seq;
    } else {
      ctx.down_circle_stamp = std::chrono::steady_clock::now();
      ctx.down_circle_offset_px = {100.0, 0.0, 0.0};
      ctx.down_circle_seq = 1;
    }
    task.tick(ctx, iface, 0.05);
    ctx.red_cross_stamp = ctx.down_circle_stamp = ctx.down_contour_stamp = stale;
    task.tick(ctx, iface, 0.2);
    assert(std::abs(ctx.position_setpoint_enu.x - 0.2) < 1e-9);
  }
  task.tick(ctx, iface, 0.2);
  // Loss exits before the 15 s total timeout and commands the saved entry pose.
  assert(std::abs(ctx.position_setpoint_enu.x) < 1e-9);
  assert(ctx.setpoint_mode == SetpointMode::POSITION);
  assert(ctx.publish_position_setpoint);
  assert(!ctx.use_position_velocity_acceleration);
  assert(std::abs(ctx.position_setpoint_enu.z - 1.0) < 1e-9);
  ctx.position_enu.x = 0.0;
  task.tick(ctx, iface, 0.05);
  // The aborted target (x=1) is suppressed briefly, but may be tried after cooldown.
  ctx.down_contour_offset_px = {100.0, 0.0, 0.0};
  ctx.red_cross_offset_px = {100.0, 0.0, 0.0};
  ctx.down_contour_stamp = ctx.red_cross_stamp = std::chrono::steady_clock::now();
  ++ctx.down_contour_seq;
  ++ctx.red_cross_seq;
  ctx.position_enu.x = 0.2;
  task.tick(ctx, iface, 0.05);
  if (red_cross) {
    // Loss is not success: red can retry immediately after return.
    assert(std::abs(ctx.position_setpoint_enu.x - 0.2) < 1e-9);
    ++ctx.red_cross_seq;
    task.tick(ctx, iface, 0.05);
    assert(ctx.position_setpoint_enu.x > 0.2);
    return;
  }
  assert(ctx.position_setpoint_enu.x < 0.2);
  for (int i = 0; i < 16; ++i) {
    ctx.down_contour_stamp = ctx.red_cross_stamp = std::chrono::steady_clock::now();
    ++ctx.down_contour_seq;
    ++ctx.red_cross_seq;
    task.tick(ctx, iface, 0.2);
  }
  assert(ctx.position_setpoint_enu.x > 0.2);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  exerciseFlow(true);
  exerciseFlow(false);
  exerciseDetectionLoss(false, false);
  exerciseDetectionLoss(false, true);
  exerciseDetectionLoss(true, false);
  exerciseDetectionLoss(true, true);
  rclcpp::shutdown();
}
