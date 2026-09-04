#include "offboard_core_pkg/tasks/arm_task.hpp"

#include <algorithm>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"

namespace offboard_core_pkg {

ArmTask::ArmTask(double timeout_s, double retry_interval_s)
    : timeout_s_(std::max(0.0, timeout_s)), retry_interval_s_(std::max(0.0, retry_interval_s)) {}

std::string ArmTask::name() const { return "arm"; }

void ArmTask::onEnter(Context &ctx, MavrosIface &) {
  elapsed_s_ = 0.0;
  retry_elapsed_s_ = 0.0;
  request_pending_ = false;
  request_accepted_ = false;
  ctx.fault.clear();
}

ITask::Status ArmTask::tick(Context &ctx, MavrosIface &iface, double dt_s) {
  if (ctx.armed) return Status::SUCCESS;
  elapsed_s_ += std::max(0.0, dt_s);
  retry_elapsed_s_ += std::max(0.0, dt_s);
  if (elapsed_s_ >= timeout_s_) { ctx.fault = "arming timed out"; return Status::FAILURE; }
  if (!request_pending_ && retry_elapsed_s_ >= retry_interval_s_) {
    retry_elapsed_s_ = 0.0;
    request_pending_ = iface.requestArm(true, [this](bool accepted) {
      request_pending_ = false;
      request_accepted_ = accepted;
    });
  }
  return Status::RUNNING;
}

}  // namespace offboard_core_pkg
