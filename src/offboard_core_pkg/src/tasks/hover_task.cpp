#include "offboard_core_pkg/tasks/hover_task.hpp"

#include <algorithm>

#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg {

HoverTask::HoverTask(double duration_s) : duration_s_(std::max(0.0, duration_s)) {}

std::string HoverTask::name() const { return "hover"; }

void HoverTask::onEnter(Context &ctx, MavrosIface &) {
  elapsed_s_ = 0.0;
  ctx.fault.clear();
}

ITask::Status HoverTask::tick(Context &, MavrosIface &, double dt_s) {
  elapsed_s_ += std::max(0.0, dt_s);
  return elapsed_s_ >= duration_s_ ? Status::SUCCESS : Status::RUNNING;
}

}  // namespace offboard_core_pkg
