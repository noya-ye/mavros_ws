#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"

namespace offboard_core_pkg {

class EgoTestNode final : public rclcpp::Node {
public:
  EgoTestNode() : Node("ego_test_node"), iface_(*this, ctx_) {
    const double rate_hz = declare_parameter<double>("setpoint_rate_hz", 20.0);
    const bool auto_start = declare_parameter<bool>("auto_start", true);
    if (rate_hz < 2.0) {
      throw std::invalid_argument("setpoint_rate_hz must be at least 2 Hz");
    }

    const auto goal_topic = declare_parameter<std::string>(
      "ego.goal_topic", "/simple_2d_planner/goal");
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

    ego_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd", rclcpp::SensorDataQoS(),
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
      "/fastlio2/lio_odom", rclcpp::SensorDataQoS(),
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

    const double presetpoint_s = declare_parameter<double>("presetpoint_duration_s", 2.0);
    const double command_timeout_s = declare_parameter<double>("command_timeout_s", 10.0);
    const double command_retry_s = declare_parameter<double>(
      "command_retry_interval_s", 1.0);
    const double takeoff_height_m = declare_parameter<double>("takeoff_height_m", 0.8);
    const double takeoff_tolerance_m = declare_parameter<double>(
      "takeoff_tolerance_m", 0.15);
    const double takeoff_timeout_s = declare_parameter<double>("takeoff_timeout_s", 30.0);
    const double hover_s = declare_parameter<double>("hover_duration_s", 5.0);
    const double land_timeout_s = declare_parameter<double>("land_timeout_s", 20.0);

    EgoGotoTask::Config ego_cfg;
    ego_cfg.task_name = "EGO_TEST_GOTO";
    ego_cfg.x_rel = declare_parameter<double>("goal.x_rel_m", 2.0);
    ego_cfg.y_rel = declare_parameter<double>("goal.y_rel_m", 0.0);
    // The EGO target is referenced to the initial MAVROS local position.
    ego_cfg.height_m = takeoff_height_m;
    ego_cfg.goal_frame = declare_parameter<std::string>("ego.goal_frame", "lidar");
    ego_cfg.planner.use_velocity_ff = declare_parameter<bool>(
      "ego.use_velocity_ff", true);
    ego_cfg.planner.use_acceleration_ff = declare_parameter<bool>(
      "ego.use_acceleration_ff", false);
    ego_cfg.planner.vel_ff_scale = declare_parameter<double>("ego.vel_ff_scale", 0.5);
    ego_cfg.planner.acc_ff_scale = declare_parameter<double>("ego.acc_ff_scale", 0.0);
    ego_cfg.planner.x_sign = declare_parameter<double>("ego.x_sign", 1.0);
    ego_cfg.planner.y_sign = declare_parameter<double>("ego.y_sign", 1.0);
    ego_cfg.planner.swap_xy = declare_parameter<bool>("ego.swap_xy", false);
    ego_cfg.planner.yaw_align_rad = declare_parameter<double>("ego.yaw_align_rad", 0.0);

    scheduler_.add(std::make_unique<PresetpointTask>(presetpoint_s));
    scheduler_.add(std::make_unique<SetOffboardTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<ArmTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<TakeoffTask>(
      takeoff_height_m, takeoff_tolerance_m, takeoff_timeout_s));
    scheduler_.add(std::make_unique<HoverTask>(hover_s));
    scheduler_.add(std::make_unique<EgoGotoTask>(
      get_logger(), get_clock(), goal_pub_, ego_cfg));
    scheduler_.add(std::make_unique<LandTask>(land_timeout_s, command_retry_s));
    scheduler_.reset();

    last_tick_ = now();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this, auto_start] {
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
    return static_cast<std::uint64_t>(stamp.sec) * 1000000ULL +
           stamp.nanosec / 1000ULL;
  }

  Context ctx_;
  MavrosIface iface_;
  Scheduler scheduler_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr ego_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
};

}  // namespace offboard_core_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_core_pkg::EgoTestNode>());
  rclcpp::shutdown();
  return 0;
}
