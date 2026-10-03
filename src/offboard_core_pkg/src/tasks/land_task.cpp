#include "offboard_core_pkg/tasks/land_task.hpp"

#include <algorithm>
#include <cmath>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"

namespace offboard_core_pkg {
namespace {
constexpr double kLandingApproachOffsetM = 0.1;
constexpr double kLandingApproachToleranceM = 0.05;
}  // namespace

LandTask::LandTask(double timeout_s, double retry_interval_s)
    : timeout_s_(std::max(0.0, timeout_s)), retry_interval_s_(std::max(0.0, retry_interval_s)) {}

std::string LandTask::name() const { return "land"; }

void LandTask::onEnter(Context &ctx, MavrosIface &) {
  elapsed_s_ = 0.0;
  retry_elapsed_s_ = 0.0;
  approach_height_m_ = 0.0;
  locked_x_m_ = 0.0;
  locked_y_m_ = 0.0;
  approach_initialized_ = false;
  horizontal_position_locked_ = false;
  request_pending_ = false;
  request_accepted_ = false;
  ctx.fault.clear();

  // Capture the horizontal reference on entry when local position is already
  // available. tick() performs the same capture if the task starts first.
  if (ctx.position_valid && ctx.finitePosition()) {
    locked_x_m_ = ctx.position_enu.x;
    locked_y_m_ = ctx.position_enu.y;
    horizontal_position_locked_ = true;
  }
}

ITask::Status LandTask::tick(Context &ctx, MavrosIface &iface, double dt_s) {
  if (!ctx.armed) return Status::SUCCESS;
  const double safe_dt_s = std::max(0.0, dt_s);
  elapsed_s_ += safe_dt_s;
  if (elapsed_s_ >= timeout_s_) { ctx.fault = "landing command timed out"; return Status::FAILURE; }

  // Approach the takeoff reference altitude before issuing the landing command.
  // home_enu is initialized from the first valid local-position measurement.
  if (!approach_initialized_) {
    if (!ctx.connected || !ctx.position_valid || !ctx.finitePosition() || !ctx.home_initialized) {
      return Status::RUNNING;
    }
    approach_height_m_ = ctx.home_enu.z + kLandingApproachOffsetM;
    if (!horizontal_position_locked_) {
      locked_x_m_ = ctx.position_enu.x;
      locked_y_m_ = ctx.position_enu.y;
      horizontal_position_locked_ = true;
    }
    approach_initialized_ = true;
  }

  // Keep the entry XY target for the entire descent and until MAVROS accepts
  // the LAND request. Do not chase the measured position while descending.
  ctx.position_setpoint_enu.x = locked_x_m_;
  ctx.position_setpoint_enu.y = locked_y_m_;
  ctx.position_setpoint_enu.z = approach_height_m_;
  ctx.yaw_setpoint_enu = ctx.home_yaw_enu;
  ctx.publish_position_setpoint = true;

  if (std::abs(ctx.position_enu.z - approach_height_m_) > kLandingApproachToleranceM) {
    return Status::RUNNING;
  }

  retry_elapsed_s_ += safe_dt_s;
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
