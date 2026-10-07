#include "offboard_core_pkg/tasks/down_drop_task.hpp"

#include <cmath>
#include <utility>

#include "offboard_core_pkg/context.hpp"
#include "../../utils/send_serial.hpp"

namespace offboard_core_pkg {

DownDropTask::DownDropTask(
  rclcpp::Logger logger, double land_height, obj_id target,
  std::string serial_device, unsigned int baud_rate)
: logger_(logger), land_height_(land_height), target_(target),
  serial_(std::make_unique<utils::SendSerial>(serial_device, baud_rate)),
  valid_config_(std::isfinite(land_height_) &&
                std::isfinite(target_.offset_x) &&
                std::isfinite(target_.offset_y) && !serial_device.empty()) {}

DownDropTask::~DownDropTask() = default;

std::string DownDropTask::name() const { return "down_drop"; }

void DownDropTask::setTarget(obj_id target) {
  target_ = target;
}

const char *DownDropTask::phaseName() const {
  switch (phase_) {
    case Phase::DOWN: return "DOWN";
    case Phase::MOVE: return "MOVE";
    case Phase::DROP: return "DROP";
    case Phase::FAILED: return "FAILED";
    case Phase::FINISHED: return "FINISHED";
  }
  return "UNKNOWN";
}

void DownDropTask::fail(Context &ctx, const std::string &reason) {
  phase_ = Phase::FAILED;
  ctx.fault = reason;
  RCLCPP_ERROR(logger_, "[DOWN_DROP] phase=%s: %s", phaseName(), reason.c_str());
}

void DownDropTask::onEnter(Context &ctx, MavrosIface &iface) {
  (void)iface;
  phase_ = Phase::DOWN;
  ctx.fault.clear();
  serial_->close();
  if (!valid_config_) {
    fail(ctx, "invalid height, offset, or serial device");
    return;
  }
  if (!ctx.position_valid || !ctx.finitePosition()) {
    fail(ctx, "local position is invalid at task entry");
    return;
  }

  goto_ = GotoTask(ctx.position_enu.x, ctx.position_enu.y, land_height_, 0.1);
  goto_.onEnter(ctx, iface);
  RCLCPP_INFO(logger_, "[DOWN_DROP] target=%d phase=%s height=%.2f",
              target_.id, phaseName(), land_height_);
}

ITask::Status DownDropTask::tick(Context &ctx, MavrosIface &iface, double dt_s) {
  if (phase_ == Phase::FAILED) return Status::FAILURE;
  if (phase_ == Phase::FINISHED) return Status::SUCCESS;
  if (!ctx.connected || !ctx.position_valid || !ctx.finitePosition()) {
    return Status::RUNNING;
  }

  if (phase_ == Phase::DOWN || phase_ == Phase::MOVE) {
    const auto result = goto_.tick(ctx, iface, dt_s);
    if (result == Status::FAILURE) {
      fail(ctx, "position target was rejected");
      return Status::FAILURE;
    }
    if (result == Status::RUNNING) return Status::RUNNING;

    if (phase_ == Phase::DOWN) {
      phase_ = Phase::MOVE;
      goto_ = GotoTask(
        ctx.position_enu.x + target_.offset_x,
        ctx.position_enu.y + target_.offset_y,
        land_height_, 0.1);
      goto_.onEnter(ctx, iface);
      RCLCPP_INFO(logger_, "[DOWN_DROP] phase=%s offset=(%.2f, %.2f)",
                  phaseName(), target_.offset_x, target_.offset_y);
      return Status::RUNNING;
    }

    phase_ = Phase::DROP;
  }

  if (phase_ == Phase::DROP) {
    // std::string error;
    // if (!serial_->open(&error)) {
    //   fail(ctx, "cannot open serial device: " + error);
    //   return Status::FAILURE;
    // }
    // constexpr char command = 'a';
    // if (!serial_->send(&command, sizeof(command), &error)) {
    //   serial_->close();
    //   fail(ctx, "cannot send drop command: " + error);
    //   return Status::FAILURE;
    // }
    // serial_->close();
    // phase_ = Phase::FINISHED;
    // RCLCPP_INFO(logger_, "[DOWN_DROP] target=%d drop command sent", target_.id);
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}

void DownDropTask::onExit(Context &, MavrosIface &) {
  serial_->close();
}

}  // namespace offboard_core_pkg
