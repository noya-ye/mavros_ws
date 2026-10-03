#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"

namespace offboard_core_pkg {

class DoorNavigationNode final : public rclcpp::Node {
public:
  DoorNavigationNode() : Node("door_navigation_node"), iface_(*this, ctx_),
                        door_input_(std::make_shared<DoorNavigationInput>()) {
    const double rate_hz = declare_parameter<double>("setpoint_rate_hz", 20.0);
    const bool auto_start = declare_parameter<bool>("auto_start", true);
    if (rate_hz < 2.0) throw std::invalid_argument("setpoint_rate_hz must be at least 2 Hz");

    const auto goal_topic = declare_parameter<std::string>(
      "ego.goal_topic", "/simple_2d_planner/goal");
    const auto cmd_topic = declare_parameter<std::string>("ego.command_topic", "/position_cmd");
    const auto odom_topic = declare_parameter<std::string>("ego.odom_topic", "/fastlio2/lio_odom");
    const auto door_status_topic = declare_parameter<std::string>("door.status_topic", "/door/status");
    const auto door_path_topic = declare_parameter<std::string>("door.path_topic", "/door/path");
    door_input_->planning_frame = declare_parameter<std::string>("door.planning_frame", "lidar");
    const double input_timeout = declare_parameter<double>("door.input_timeout_s", 0.6);
    if (input_timeout <= 0.0) throw std::invalid_argument("door.input_timeout_s must be positive");

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    ego_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      cmd_topic, rclcpp::SensorDataQoS(),
      [this](quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg) {
        ctx_.ego_cmd_position = {msg->position.x, msg->position.y, msg->position.z};
        ctx_.ego_cmd_velocity = {msg->velocity.x, msg->velocity.y, msg->velocity.z};
        ctx_.ego_cmd_acceleration = {
          msg->acceleration.x, msg->acceleration.y, msg->acceleration.z};
        ctx_.ego_cmd_yaw = msg->yaw;
        ctx_.ego_cmd_valid = true;
        ctx_.ego_cmd_stamp_us = stampUs(msg->header.stamp);
      });
    ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        ctx_.ego_odom_position = {
          msg->pose.pose.position.x, msg->pose.pose.position.y,
          msg->pose.pose.position.z};
        ctx_.ego_odom_velocity = {
          msg->twist.twist.linear.x, msg->twist.twist.linear.y,
          msg->twist.twist.linear.z};
        ctx_.ego_odom_valid = true;
        ctx_.ego_odom_stamp_us = stampUs(msg->header.stamp);
      });
    door_status_sub_ = create_subscription<std_msgs::msg::String>(
      door_status_topic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { updateStatus(msg->data); });
    door_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      door_path_topic, 10,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        door_input_->path = *msg;
        door_input_->path_received = std::chrono::steady_clock::now();
      });

    const double presetpoint_s = declare_parameter<double>("presetpoint_duration_s", 2.0);
    const double command_timeout_s = declare_parameter<double>("command_timeout_s", 10.0);
    const double command_retry_s = declare_parameter<double>("command_retry_interval_s", 1.0);
    const double takeoff_height_m = declare_parameter<double>("takeoff_height_m", 0.8);
    const double takeoff_tolerance_m = declare_parameter<double>("takeoff_tolerance_m", 0.15);
    const double takeoff_timeout_s = declare_parameter<double>("takeoff_timeout_s", 30.0);
    const double hover_s = declare_parameter<double>("hover_duration_s", 3.0);
    const double land_timeout_s = declare_parameter<double>("land_timeout_s", 20.0);

    EgoVelPlanner::Config planner_cfg;
    planner_cfg.use_velocity_ff = declare_parameter<bool>("ego.use_velocity_ff", true);
    planner_cfg.use_acceleration_ff = declare_parameter<bool>("ego.use_acceleration_ff", false);
    planner_cfg.vel_ff_scale = declare_parameter<double>("ego.vel_ff_scale", 0.5);
    planner_cfg.acc_ff_scale = declare_parameter<double>("ego.acc_ff_scale", 0.0);
    planner_cfg.x_sign = declare_parameter<double>("ego.x_sign", 1.0);
    planner_cfg.y_sign = declare_parameter<double>("ego.y_sign", 1.0);
    planner_cfg.swap_xy = declare_parameter<bool>("ego.swap_xy", false);
    planner_cfg.yaw_align_rad = declare_parameter<double>("ego.yaw_align_rad", 0.0);

    scheduler_.add(std::make_unique<PresetpointTask>(presetpoint_s));
    scheduler_.add(std::make_unique<SetOffboardTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<ArmTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<TakeoffTask>(
      takeoff_height_m, takeoff_tolerance_m, takeoff_timeout_s));
    scheduler_.add(std::make_unique<HoverTask>(hover_s));
    scheduler_.add(std::make_unique<DoorNavigationTask>(
      get_logger(), goal_pub_, door_input_, planner_cfg, input_timeout));
    scheduler_.add(std::make_unique<LandTask>(land_timeout_s, command_retry_s));
    scheduler_.reset();

    last_tick_ = now();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this, auto_start] {
        const auto current = now();
        const double dt_s = std::clamp((current - last_tick_).seconds(), 0.0, 0.2);
        last_tick_ = current;
        if (auto_start && !scheduler_.done() && !scheduler_.failed()) {
          scheduler_.tick(ctx_, iface_, dt_s);
        }
        iface_.publishSetpoint();
      });
  }

private:
  static std::uint64_t stampUs(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<std::uint64_t>(stamp.sec) * 1000000ULL + stamp.nanosec / 1000ULL;
  }

  void updateStatus(const std::string &data) {
    try {
      const auto status = nlohmann::json::parse(data);
      door_input_->state = status.value("state", std::string{});
      door_input_->stage = status.value("plan_stage", std::string{});
      door_input_->path_kind = status.value("path_kind", std::string{});
      door_input_->segment_clear = status.value("segment_clear", false);
      door_input_->preview_mode = status.value("preview_mode", false);
      door_input_->status_valid = true;
      door_input_->status_received = std::chrono::steady_clock::now();
    } catch (const std::exception &e) {
      door_input_->status_valid = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Ignoring malformed door status: %s", e.what());
    }
  }

  Context ctx_;
  MavrosIface iface_;
  Scheduler scheduler_;
  std::shared_ptr<DoorNavigationInput> door_input_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr ego_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr door_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr door_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
};

}  // namespace offboard_core_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_core_pkg::DoorNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
