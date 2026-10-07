#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cmath>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/tasks/red_cross_align_task.hpp"

namespace offboard_core_pkg {
class MavrosIface {};
}

using offboard_core_pkg::Context;
using offboard_core_pkg::ITask;
using offboard_core_pkg::MavrosIface;
using offboard_core_pkg::RedCrossAlignTask;

int main() {
  MavrosIface iface;
  Context ctx;
  ctx.connected = true;
  ctx.position_valid = true;
  ctx.position_enu = {2.0, 3.0, 1.0};
  ctx.yaw_enu = std::acos(-1.0) / 2.0;
  ctx.home_yaw_enu = 0.2;

  RedCrossAlignTask task(100.0, 2, 0.1, 0.2);
  task.onEnter(ctx, iface);
  ctx.red_cross_offset_px = {100.0, 0.0, 0.0};
  ctx.red_cross_stamp = std::chrono::steady_clock::now();
  ++ctx.red_cross_seq;
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  assert(std::abs(ctx.position_setpoint_enu.x - 2.0) < 1e-9);
  assert(std::abs(ctx.position_setpoint_enu.y - 3.2) < 1e-9);
  assert(ctx.position_setpoint_enu.z == 1.0);
  assert(ctx.yaw_setpoint_enu == ctx.home_yaw_enu);

  ctx.red_cross_offset_px = {0.0, 5.0, 0.0};
  ctx.red_cross_stamp = std::chrono::steady_clock::now();
  ++ctx.red_cross_seq;
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::RUNNING);
  ctx.red_cross_stamp = std::chrono::steady_clock::now();
  ++ctx.red_cross_seq;
  assert(task.tick(ctx, iface, 0.05) == ITask::Status::SUCCESS);

  RedCrossAlignTask invalid(0.0, 0, 0.1, 0.2);
  invalid.onEnter(ctx, iface);
  assert(invalid.tick(ctx, iface, 0.05) == ITask::Status::FAILURE);
  assert(!ctx.fault.empty());
}
