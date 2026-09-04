#include "offboard_core_pkg/tasks/takeoff_task.hpp"

#include <algorithm>
#include <cmath>

#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg {

TakeoffTask::TakeoffTask(double height_m, double tolerance_m, double timeout_s)
    : height_m_(std::max(0.0, height_m)),
      tolerance_m_(std::max(0.01, tolerance_m)),
      timeout_s_(std::max(0.0, timeout_s)) {}

std::string TakeoffTask::name() const { return "takeoff"; }

void TakeoffTask::onEnter(Context &ctx, MavrosIface &) {
  elapsed_s_ = 0.0;
  target_initialized_ = false;
  ctx.fault.clear();
}

ITask::Status TakeoffTask::tick(Context &ctx, MavrosIface &, double dt_s) {
  elapsed_s_ += std::max(0.0, dt_s);
  if (elapsed_s_ >= timeout_s_) {
    ctx.fault = "takeoff timed out";
    return Status::FAILURE;
  }
  if (!ctx.connected || !ctx.position_valid || !ctx.armed) return Status::RUNNING;

  if (!target_initialized_) {
    // PresetpointTask has already captured the safe XY/yaw reference.
    ctx.position_setpoint_enu.z += height_m_;
    target_initialized_ = true;
  }

  const double altitude_error = std::abs(ctx.position_enu.z - ctx.position_setpoint_enu.z);
  return altitude_error <= tolerance_m_ ? Status::SUCCESS : Status::RUNNING;
}

}  // namespace offboard_core_pkg
