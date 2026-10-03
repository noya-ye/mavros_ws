#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/tasks/align_down_task.hpp"

namespace offboard_core_pkg {
class MavrosIface {};
}

using offboard_core_pkg::AlignDownTask;
using offboard_core_pkg::Context;
using offboard_core_pkg::ITask;
using offboard_core_pkg::MavrosIface;

int main() {
  MavrosIface iface;
  Context ctx;
  ctx.connected = true;
  ctx.position_valid = true;
  ctx.position_enu = {2.0, 3.0, 1.0};
  ctx.yaw_enu = std::acos(-1.0) / 2.0;
  ctx.home_yaw_enu = 0.3;
  AlignDownTask task(100.0, 2, 0.1, 0.2);
  task.onEnter(ctx, iface);
  assert(ctx.yaw_setpoint_enu == ctx.home_yaw_enu);

  const auto frame = [&](double x, double y) {
    ctx.down_circle_offset_px = {x, y, 0.0};
    ctx.down_circle_stamp = std::chrono::steady_clock::now();
    ++ctx.down_circle_seq;
  };
  frame(100.0, 0.0);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(std::abs(ctx.position_setpoint_enu.x - 2.0) < 1e-9);
  assert(std::abs(ctx.position_setpoint_enu.y - 3.2) < 1e-9);
  assert(ctx.position_setpoint_enu.z == 1.0);
  assert(ctx.yaw_setpoint_enu == ctx.home_yaw_enu);

  // Clamp the vector magnitude while preserving its direction.
  ctx.position_enu.z = 1.4;
  frame(30.0, 40.0);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(std::abs(ctx.position_setpoint_enu.x - 1.84) < 1e-9);
  assert(std::abs(ctx.position_setpoint_enu.y - 3.12) < 1e-9);
  assert(ctx.position_setpoint_enu.z == 1.0);

  // An error above arrival tolerance but below max_step is used in full.
  frame(15.0, 0.0);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(std::abs(ctx.position_setpoint_enu.y - 3.15) < 1e-9);

  ctx.yolo_detections = {{1, 0.81F}};
  ctx.yolo_detections_stamp = std::chrono::steady_clock::now();
  ++ctx.yolo_detections_seq;
  frame(0.0, 5.0);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(ctx.position_setpoint_enu.z == 1.0);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  // A confirmed target is checked only once, even if later YOLO frames are empty.
  ctx.yolo_detections.clear();
  frame(0.0, 5.0);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::SUCCESS);

  task.onEnter(ctx, iface);
  ctx.down_circle_stamp = std::chrono::steady_clock::now() -
                          std::chrono::seconds(1);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(ctx.position_setpoint_enu.x == ctx.position_enu.x);
  assert(ctx.position_setpoint_enu.z == 1.4);
  ctx.down_contour_offset_px = {0.0, -100.0, 0.0};
  ctx.down_contour_stamp = std::chrono::steady_clock::now();
  ++ctx.down_contour_seq;
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(std::abs(ctx.position_setpoint_enu.x - 2.2) < 1e-9);
  assert(ctx.yaw_setpoint_enu == ctx.home_yaw_enu);

  task.onPause(ctx, iface);
  ctx.position_enu.z = 1.8;
  task.onResume(ctx, iface);
  assert(ctx.position_setpoint_enu.z == 1.4);
  assert(ctx.yaw_setpoint_enu == ctx.home_yaw_enu);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(ctx.position_setpoint_enu.x == ctx.position_enu.x);

  AlignDownTask no_target(100.0, 2, 0.1, 0.2);
  no_target.onEnter(ctx, iface);
  ctx.yolo_detections.clear();
  ctx.yolo_detections_stamp = std::chrono::steady_clock::now();
  ++ctx.yolo_detections_seq;
  frame(0.0, 0.0);
  assert(no_target.tick(ctx, iface, 0.05) == ITask::Status::SUCCESS);
  assert(no_target.reached_arrival_tolerance());
  assert(ctx.fault.empty());
  assert(ctx.position_setpoint_enu.x == ctx.position_enu.x);
  assert(ctx.position_setpoint_enu.y == ctx.position_enu.y);
  // A skipped task remains complete until onEnter starts a new invocation.
  ctx.yolo_detections = {{0, 0.99F}};
  frame(0.0, 0.0);
  assert(no_target.tick(ctx, iface, 0.05) == ITask::Status::SUCCESS);

  AlignDownTask contour_matches_circle(100.0, 1, 0.1, 0.2);
  ctx.down_circle_offset_px = {40.0, 0.0, 0.0};
  ctx.down_circle_stamp = std::chrono::steady_clock::now() -
                          std::chrono::seconds(1);
  contour_matches_circle.onEnter(ctx, iface);
  ctx.yolo_detections = {{0, 0.99F}};
  ctx.yolo_detections_stamp = std::chrono::steady_clock::now();
  ++ctx.yolo_detections_seq;
  ctx.down_contour_offset_px = {5.0, 0.0, 0.0};
  ctx.down_contour_stamp = std::chrono::steady_clock::now();
  ++ctx.down_contour_seq;
  assert(contour_matches_circle.tick(ctx, iface, 0.05) ==
         ITask::Status::SUCCESS);

  Context contour_only_ctx;
  contour_only_ctx.connected = true;
  contour_only_ctx.position_valid = true;
  contour_only_ctx.position_enu = {0.0, 0.0, 1.0};
  contour_only_ctx.yaw_enu = 0.0;
  contour_only_ctx.home_yaw_enu = 0.0;
  contour_only_ctx.yolo_detections = {{1, 0.81F}};
  contour_only_ctx.yolo_detections_stamp = std::chrono::steady_clock::now();
  contour_only_ctx.yolo_detections_seq = 1;
  AlignDownTask contour_only(800.0, 2, 0.08, 0.15);
  contour_only.onEnter(contour_only_ctx, iface);
  const auto contour_frame = [&]() {
    contour_only_ctx.down_contour_offset_px = {5.0, 0.0, 0.0};
    contour_only_ctx.down_contour_stamp = std::chrono::steady_clock::now();
    ++contour_only_ctx.down_contour_seq;
  };
  contour_frame();
  assert(contour_only.tick(contour_only_ctx, iface, 0.05) ==
         ITask::Status::RUNNING);
  assert(contour_only.reached_arrival_tolerance());
  contour_frame();
  assert(contour_only.tick(contour_only_ctx, iface, 0.05) ==
         ITask::Status::SUCCESS);

  for (int scenario = 0; scenario < 6; ++scenario) {
    no_target.onEnter(ctx, iface);
    ctx.yolo_detections = {{0, 0.99F}};
    ctx.yolo_detections_stamp = std::chrono::steady_clock::now();
    ctx.yolo_detections_seq = 1;
    if (scenario == 0) ctx.yolo_detections[0].confidence = 0.40F;
    if (scenario == 1) ctx.yolo_detections[0].confidence = 0.39F;
    if (scenario == 2) ctx.yolo_detections_stamp -= std::chrono::seconds(1);
    if (scenario == 3) ctx.yolo_detections_stamp += std::chrono::seconds(1);
    if (scenario == 4) ctx.yolo_detections_seq = 0;
    if (scenario == 5) ctx.yolo_detections[0].confidence =
        std::numeric_limits<float>::quiet_NaN();
    frame(0.0, 0.0);
    assert(no_target.tick(ctx, iface, 0.05) == ITask::Status::SUCCESS);
    assert(ctx.fault.empty());
  }

  no_target.onEnter(ctx, iface);
  ctx.yolo_detections = {{0, 0.20F}, {1, 0.41F}};
  ctx.yolo_detections_stamp = std::chrono::steady_clock::now();
  ctx.yolo_detections_seq = 1;
  frame(0.0, 0.0);
  assert(no_target.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  ctx.yolo_detections.clear();
  // Leaving and re-entering tolerance resets stable frames, but not target confirmation.
  frame(100.0, 0.0);
  assert(no_target.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  no_target.onPause(ctx, iface);
  no_target.onResume(ctx, iface);
  frame(0.0, 0.0);
  assert(no_target.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  frame(0.0, 0.0);
  assert(no_target.tick(ctx, iface, 0.05) == ITask::Status::SUCCESS);

  AlignDownTask invalid(0.0, 0, -1.0, 0.2);
  invalid.onEnter(ctx, iface);
  assert(invalid.tick(ctx, iface, 0.05) == ITask::Status::FAILURE);
  assert(!ctx.fault.empty());

  for (const double max_step : {0.0, -0.1,
       std::numeric_limits<double>::infinity(),
       std::numeric_limits<double>::quiet_NaN()}) {
    AlignDownTask invalid_step(100.0, 2, 0.1, max_step);
    invalid_step.onEnter(ctx, iface);
    assert(invalid_step.tick(ctx, iface, 0.05) == ITask::Status::FAILURE);
    assert(!ctx.fault.empty());
  }
}
