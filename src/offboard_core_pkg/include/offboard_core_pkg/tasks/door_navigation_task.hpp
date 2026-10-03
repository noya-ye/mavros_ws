#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/planners/ego_vel_planner.hpp"

namespace offboard_core_pkg {

struct DoorNavigationInput {
  nav_msgs::msg::Path path;
  std::string state;
  std::string stage;
  std::string path_kind;
  std::string planning_frame{"lidar"};
  bool segment_clear{false};
  bool preview_mode{true};
  bool status_valid{false};
  std::chrono::steady_clock::time_point status_received{};
  std::chrono::steady_clock::time_point path_received{};
};

class DoorNavigationTask final : public ITask {
public:
  DoorNavigationTask(rclcpp::Logger logger,
                     rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
                     std::shared_ptr<DoorNavigationInput> input,
                     const EgoVelPlanner::Config &planner_cfg,
                     double input_timeout_s);
  std::string name() const override { return "DOOR_NAVIGATION"; }
  void onEnter(Context &ctx, MavrosIface &) override;
  Status tick(Context &ctx, MavrosIface &, double dt_s) override;
  void onExit(Context &ctx, MavrosIface &) override;

private:
  void hold(Context &ctx) const;
  bool publishTarget(const nav_msgs::msg::Path &path, std::size_t index);

  rclcpp::Logger logger_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  std::shared_ptr<DoorNavigationInput> input_;
  EgoVelPlanner planner_;
  double input_timeout_s_;
  std::string last_stage_;
  geometry_msgs::msg::Point last_target_;
  bool target_valid_{false};
};

}  // namespace offboard_core_pkg
