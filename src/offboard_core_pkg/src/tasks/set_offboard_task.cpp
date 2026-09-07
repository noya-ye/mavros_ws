#include "offboard_core_pkg/tasks/set_offboard_task.hpp"

#include <algorithm>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"

namespace offboard_core_pkg {

SetOffboardTask::SetOffboardTask(double timeout_s, double retry_interval_s)
    : timeout_s_(std::max(0.0, timeout_s)), retry_interval_s_(std::max(0.0, retry_interval_s)) {}

std::string SetOffboardTask::name() const { return "set_offboard"; }

void SetOffboardTask::onEnter(Context &ctx, MavrosIface &) {
  elapsed_s_ = 0.0;
  retry_elapsed_s_ = 0.0;
  request_pending_ = false;//是否已经有MAVROS请求正在进行中
  request_accepted_ = false;//是否已经有MAVROS请求被接受
  ctx.fault.clear();
}

ITask::Status SetOffboardTask::tick(Context &ctx, MavrosIface &iface, double dt_s) {
  if (ctx.mode == "OFFBOARD") return Status::SUCCESS;
  elapsed_s_ += std::max(0.0, dt_s);
  retry_elapsed_s_ += std::max(0.0, dt_s);
  if (elapsed_s_ >= timeout_s_) { ctx.fault = "OFFBOARD mode request timed out"; return Status::FAILURE; }
  if (!request_pending_ && retry_elapsed_s_ >= retry_interval_s_) {
    retry_elapsed_s_ = 0.0;
    request_pending_ = iface.requestMode("OFFBOARD", [this](bool accepted) {
      request_pending_ = false;
      request_accepted_ = accepted;
    });//发送一次请求，异步等待MAVROS的响应，响应结果会通过回调函数设置request_accepted_标志
  }//若请求已经发送过了，则等待MAVROS的响应，直到超时或者成功切换到OFFBOARD模式
  return Status::RUNNING;
}

}  // namespace offboard_core_pkg
