#include "offboard_core_pkg/tasks/land_task.hpp"

#include <algorithm>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"

namespace offboard_core_pkg {

LandTask::LandTask(double timeout_s, double retry_interval_s)
    : timeout_s_(std::max(0.0, timeout_s)), retry_interval_s_(std::max(0.0, retry_interval_s)) {}

std::string LandTask::name() const { return "land"; }

void LandTask::onEnter(Context &ctx, MavrosIface &) {
  elapsed_s_ = 0.0;
  retry_elapsed_s_ = 0.0;
  request_pending_ = false;
  request_accepted_ = false;
  ctx.fault.clear();
}

ITask::Status LandTask::tick(Context &ctx, MavrosIface &iface, double dt_s) {
  if (!ctx.armed) return Status::SUCCESS;
  elapsed_s_ += std::max(0.0, dt_s);
  retry_elapsed_s_ += std::max(0.0, dt_s);
  if (elapsed_s_ >= timeout_s_) { ctx.fault = "landing command timed out"; return Status::FAILURE; }
  if (!request_pending_ && retry_elapsed_s_ >= retry_interval_s_) {
    retry_elapsed_s_ = 0.0;
    request_pending_ = iface.requestLand([this](bool accepted) {
      request_pending_ = false;
      request_accepted_ = accepted;
    });
  }
  return Status::RUNNING;
}

}  // namespace offboard_core_pkg
