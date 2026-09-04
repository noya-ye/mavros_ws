#include "offboard_core_pkg/tasks/presetpoint_task.hpp"

#include <algorithm>

#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg {

PresetpointTask::PresetpointTask(double duration_s) : duration_s_(std::max(0.0, duration_s)) {}

std::string PresetpointTask::name() const { return "presetpoint"; }

void PresetpointTask::onEnter(Context &ctx, MavrosIface &) {
  elapsed_s_ = 0.0;
  target_initialized_ = false;
  ctx.fault.clear();
}

ITask::Status PresetpointTask::tick(Context &ctx, MavrosIface &, double dt_s) {
  if (!ctx.connected || !ctx.position_valid) return Status::RUNNING;
  if (!target_initialized_) {
    ctx.position_setpoint_enu = ctx.position_enu;
    ctx.yaw_setpoint_enu = ctx.yaw_enu;
    target_initialized_ = true;
  }
  elapsed_s_ += std::max(0.0, dt_s);
  return elapsed_s_ >= duration_s_ ? Status::SUCCESS : Status::RUNNING;
}

}  // namespace offboard_core_pkg
