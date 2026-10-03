#include "offboard_core_pkg/tasks/door_navigation_task.hpp"

#include <cmath>

namespace offboard_core_pkg {

DoorNavigationTask::DoorNavigationTask(
    rclcpp::Logger logger,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
    std::shared_ptr<DoorNavigationInput> input,
    const EgoVelPlanner::Config &planner_cfg, double input_timeout_s)
    : logger_(logger), goal_pub_(std::move(goal_pub)),
      input_(std::move(input)), planner_(planner_cfg),
      input_timeout_s_(input_timeout_s) {}

void DoorNavigationTask::onEnter(Context &ctx, MavrosIface &) {
  planner_.reset(ctx);
  ctx.ego_cmd_valid = false;
  target_valid_ = false;
  last_stage_.clear();
}

DoorNavigationTask::Status DoorNavigationTask::tick(
    Context &ctx, MavrosIface &, double) {
  if (!ctx.position_valid || !ctx.finitePosition()) {
    hold(ctx);
    return Status::RUNNING;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto status_age = std::chrono::duration<double>(now - input_->status_received).count();
  const auto path_age = std::chrono::duration<double>(now - input_->path_received).count();
  const auto &path = input_->path;
  if (input_->status_valid && status_age <= input_timeout_s_ &&
      input_->state == "COMPLETE") {
    hold(ctx);
    return Status::SUCCESS;
  }
  const bool phase_path_valid =
      (input_->stage == "APPROACH" && path.poses.size() == 3) ||
      (input_->stage == "CROSS" && path.poses.size() == 2);
  const bool usable = input_->status_valid && status_age <= input_timeout_s_ &&
      path_age <= input_timeout_s_ && input_->preview_mode &&
      input_->path_kind == "geometric_reference" && input_->segment_clear &&
      (input_->state == "PREVIEW") && phase_path_valid &&
      path.header.frame_id == input_->planning_frame;
  if (!usable) {
    hold(ctx);
    target_valid_ = false;
    return Status::RUNNING;
  }

  // APPROACH paths are [current pose, pre-gate, post-gate]; CROSS paths are
  // [current pose, post-gate]. The selected point is always the active phase end.
  const std::size_t target_index = 1;
  if (!publishTarget(path, target_index)) {
    hold(ctx);
    target_valid_ = false;
    return Status::RUNNING;
  }

  const auto result = planner_.plan(ctx);
  if (result != EgoVelPlanner::Result::OK) {
    RCLCPP_DEBUG(logger_, "Door EGO tracking held: %s",
                 EgoVelPlanner::resultName(result));
  }
  return Status::RUNNING;
}

void DoorNavigationTask::onExit(Context &ctx, MavrosIface &) {
  hold(ctx);
  target_valid_ = false;
}

void DoorNavigationTask::hold(Context &ctx) const {
  ctx.position_setpoint_enu = ctx.position_enu;
  ctx.velocity_setpoint_enu = {0.0, 0.0, 0.0};
  ctx.acceleration_setpoint_enu = {0.0, 0.0, 0.0};
  ctx.yaw_setpoint_enu = ctx.yaw_enu;
  ctx.setpoint_mode = SetpointMode::POSITION;
  ctx.use_position_velocity_acceleration = false;
  ctx.publish_position_setpoint = true;
}

bool DoorNavigationTask::publishTarget(const nav_msgs::msg::Path &path,
                                      std::size_t index) {
  if (index >= path.poses.size()) return false;
  const auto &pose = path.poses[index];
  const auto &p = pose.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
  const bool changed = !target_valid_ || input_->stage != last_stage_ ||
      std::hypot(p.x - last_target_.x, p.y - last_target_.y) > 0.05 ||
      std::abs(p.z - last_target_.z) > 0.05;
  if (changed) {
    auto goal = pose;
    goal.header = path.header;
    goal_pub_->publish(goal);
    last_target_ = p;
    last_stage_ = input_->stage;
    target_valid_ = true;
  }
  return true;
}

}  // namespace offboard_core_pkg
