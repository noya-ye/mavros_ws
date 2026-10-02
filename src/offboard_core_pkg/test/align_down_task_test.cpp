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

  frame(0.0, 5.0);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(ctx.position_setpoint_enu.z == 1.0);
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
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
