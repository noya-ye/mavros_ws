#include "offboard_core_pkg/tasks/goto_task.hpp"

#include <algorithm>
#include <cmath>

#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg {

GotoTask::GotoTask(double x,double y,double z,double tolerance_m)
    : x_(x), y_(y), z_(z), tolerance_m_(tolerance_m) {
  valid_target_ = std::isfinite(x_) && std::isfinite(y_) && std::isfinite(z_) &&
                  std::isfinite(tolerance_m_) && tolerance_m_ > 0.0;
}

std::string GotoTask::name() const { return "goto"; }

void GotoTask::onEnter(Context &ctx, MavrosIface &) {
  ctx.fault.clear();
  if (!valid_target_) {
    ctx.fault = "goto target or tolerance is invalid";
    return;
  }
  ctx.position_setpoint_enu = {x_, y_, z_};
}

ITask::Status GotoTask::tick(Context &ctx, MavrosIface &, double) {
  if (!valid_target_) return Status::FAILURE;
  if (!ctx.connected || !ctx.position_valid) return Status::RUNNING;

  const double dx = ctx.position_enu.x - x_;
  const double dy = ctx.position_enu.y - y_;
  const double dz = ctx.position_enu.z - z_;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  return distance <= tolerance_m_ ? Status::SUCCESS : Status::RUNNING;
}

}  // namespace offboard_core_pkg
