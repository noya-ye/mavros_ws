#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"

namespace offboard_core_pkg {

class CorridorDoorNode final : public rclcpp::Node {
public:
  CorridorDoorNode() : Node("corridor_door_node"), iface_(*this, ctx_) {
    const double rate = declare_parameter<double>("setpoint_rate_hz", 20.0);
    const bool auto_start = declare_parameter<bool>("auto_start", true);
    if (!std::isfinite(rate) || rate < 2.0) {
      throw std::invalid_argument("setpoint_rate_hz must be at least 2 Hz");
    }
    const double warmup_s = declare_parameter<double>("presetpoint_duration_s", 2.0);
    const double command_timeout_s = declare_parameter<double>("command_timeout_s", 10.0);
    const double command_retry_s = declare_parameter<double>("command_retry_interval_s", 1.0);
    const double takeoff_height = declare_parameter<double>("takeoff.height_m", 0.8);
    const double takeoff_tolerance = declare_parameter<double>("takeoff.tolerance_m", 0.12);
    const double takeoff_timeout = declare_parameter<double>("takeoff.timeout_s", 30.0);
    const double land_timeout = declare_parameter<double>("land.timeout_s", 20.0);
    const std::string goal_topic = declare_parameter<std::string>(
        "ego.goal_topic", "/simple_2d_planner/goal");
    const std::string grid_topic = declare_parameter<std::string>(
        "ego.occupancy_grid_topic", "/ego_2d_planner/occupancy_grid");
    const std::string cmd_topic = declare_parameter<std::string>(
        "ego.command_topic", "/position_cmd");
    const std::string odom_topic = declare_parameter<std::string>(
        "ego.odom_topic", "/fastlio2/lio_odom");

    CorridorDoorTask::Config cfg;
    cfg.door_count = declare_parameter<int>("corridor.door_count", 1);
    cfg.occupied_threshold = declare_parameter<int>("corridor.occupied_threshold", 50);
    cfg.min_opening_width_m = declare_parameter<double>("corridor.min_opening_width_m", 0.5);
    cfg.max_opening_width_m = declare_parameter<double>("corridor.max_opening_width_m", 1.5);
    cfg.min_lookahead_m = declare_parameter<double>("corridor.min_lookahead_m", 0.5);
    cfg.max_lookahead_m = declare_parameter<double>("corridor.max_lookahead_m", 5.0);
    cfg.occupancy_timeout_s = declare_parameter<double>("corridor.occupancy_timeout_s", 0.5);
    cfg.goto_tolerance_m = declare_parameter<double>("corridor.goto_tolerance_m", 0.08);
    cfg.crossing_distance_m = declare_parameter<double>("corridor.crossing_distance_m", 0.1);
    cfg.stage_timeout_s = declare_parameter<double>("corridor.stage_timeout_s", 30.0);
    if (cfg.door_count <= 0 || cfg.occupied_threshold < 1 ||
        !std::isfinite(cfg.min_opening_width_m) || cfg.min_opening_width_m <= 0.0 ||
        !std::isfinite(cfg.max_opening_width_m) ||
        cfg.max_opening_width_m <= cfg.min_opening_width_m ||
        !std::isfinite(cfg.min_lookahead_m) || cfg.min_lookahead_m < 0.0 ||
        !std::isfinite(cfg.max_lookahead_m) || cfg.max_lookahead_m <= cfg.min_lookahead_m ||
        !std::isfinite(cfg.occupancy_timeout_s) || cfg.occupancy_timeout_s <= 0.0 ||
        !std::isfinite(cfg.goto_tolerance_m) || cfg.goto_tolerance_m <= 0.0 ||
        !std::isfinite(cfg.crossing_distance_m) || cfg.crossing_distance_m <= 0.0 ||
        !std::isfinite(cfg.stage_timeout_s) || cfg.stage_timeout_s <= 0.0) {
      throw std::invalid_argument("corridor parameters are invalid");
    }
    cfg.ego.task_name = "CORRIDOR_DOOR_EGO";
    cfg.ego.height_m = takeoff_height;
    cfg.ego.goal_frame = declare_parameter<std::string>("ego.goal_frame", "camera_init");
    cfg.ego.arrive_xy_m = declare_parameter<double>("ego.arrive_xy_m", 0.15);
    cfg.ego.arrive_z_m = declare_parameter<double>("ego.arrive_z_m", 0.15);
    cfg.ego.stable_vxy_mps = declare_parameter<double>("ego.stable_vxy_mps", 0.15);
    cfg.ego.stable_vz_mps = declare_parameter<double>("ego.stable_vz_mps", 0.12);
    cfg.ego.stable_required_s = declare_parameter<double>("ego.stable_required_s", 0.4);
    cfg.ego.planner.use_velocity_ff = declare_parameter<bool>("ego.use_velocity_ff", true);
    cfg.ego.planner.use_acceleration_ff = declare_parameter<bool>("ego.use_acceleration_ff", false);
    cfg.ego.planner.vel_ff_scale = declare_parameter<double>("ego.vel_ff_scale", 0.5);
    cfg.ego.planner.acc_ff_scale = declare_parameter<double>("ego.acc_ff_scale", 0.0);
    cfg.ego.planner.x_sign = declare_parameter<double>("ego.x_sign", 1.0);
    cfg.ego.planner.y_sign = declare_parameter<double>("ego.y_sign", 1.0);
    cfg.ego.planner.swap_xy = declare_parameter<bool>("ego.swap_xy", false);
    cfg.ego.planner.yaw_align_rad = declare_parameter<double>("ego.yaw_align_rad", 0.0);

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    ego_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
        cmd_topic, rclcpp::SensorDataQoS(), [this](quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg) {
          ctx_.ego_cmd_position = {msg->position.x, msg->position.y, msg->position.z};
          ctx_.ego_cmd_velocity = {msg->velocity.x, msg->velocity.y, msg->velocity.z};
          ctx_.ego_cmd_acceleration = {msg->acceleration.x, msg->acceleration.y, msg->acceleration.z};
          ctx_.ego_cmd_yaw = msg->yaw;
          ctx_.ego_cmd_valid = true;
          ctx_.ego_cmd_stamp_us = stampUs(msg->header.stamp);
        });
    ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          ctx_.ego_odom_position = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                                    msg->pose.pose.position.z};
          ctx_.ego_odom_velocity = {msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                    msg->twist.twist.linear.z};
          ctx_.ego_odom_valid = true;
          ctx_.ego_odom_stamp_us = stampUs(msg->header.stamp);
        });
    occupancy_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        grid_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          ctx_.occupancy_grid_resolution = msg->info.resolution;
          ctx_.occupancy_grid_origin_x = msg->info.origin.position.x;
          ctx_.occupancy_grid_origin_y = msg->info.origin.position.y;
          ctx_.occupancy_grid_width = msg->info.width;
          ctx_.occupancy_grid_height = msg->info.height;
          ctx_.occupancy_grid_data = msg->data;
          ctx_.occupancy_grid_stamp_us = stampUs(msg->header.stamp);
          if (ctx_.occupancy_grid_stamp_us == 0) {
            ctx_.occupancy_grid_stamp_us = static_cast<std::uint64_t>(now().nanoseconds() / 1000ULL);
          }
          ctx_.occupancy_grid_valid = ctx_.occupancy_grid_data.size() ==
              static_cast<std::size_t>(msg->info.width) * msg->info.height;
        });

    scheduler_.add(std::make_unique<PresetpointTask>(warmup_s));
    scheduler_.add(std::make_unique<SetOffboardTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<ArmTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<TakeoffTask>(takeoff_height, takeoff_tolerance, takeoff_timeout));
    scheduler_.add(std::make_unique<CorridorDoorTask>(get_logger(), get_clock(), goal_pub_, cfg));
    scheduler_.add(std::make_unique<LandTask>(land_timeout, command_retry_s));
    scheduler_.reset();

    last_tick_ = now();
    const auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this, auto_start] {
          const auto current = now();
          const double dt = std::clamp((current - last_tick_).seconds(), 0.0, 0.2);
          last_tick_ = current;
          if (auto_start && !scheduler_.done() && !scheduler_.failed()) {
            scheduler_.tick(ctx_, iface_, dt);
          }
          iface_.publishSetpoint();
        });
  }

private:
  static std::uint64_t stampUs(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<std::uint64_t>(stamp.sec) * 1000000ULL + stamp.nanosec / 1000ULL;
  }

  Context ctx_;
  MavrosIface iface_;
  Scheduler scheduler_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr ego_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
};

}  // namespace offboard_core_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_core_pkg::CorridorDoorNode>());
  rclcpp::shutdown();
  return 0;
}
