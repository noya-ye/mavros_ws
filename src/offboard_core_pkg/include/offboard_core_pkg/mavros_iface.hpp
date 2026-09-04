#pragma once

#include <functional>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg {
class MavrosIface {
public:
  using CommandCallback = std::function<void(bool)>;
  explicit MavrosIface(rclcpp::Node &node, Context &context);
  void publishSetpoint();
  bool requestMode(const std::string &mode, CommandCallback callback);
  bool requestArm(bool arm, CommandCallback callback);
  bool requestLand(CommandCallback callback);

private:
  rclcpp::Node &node_;
  Context &ctx_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
};
}  // namespace offboard_core_pkg
