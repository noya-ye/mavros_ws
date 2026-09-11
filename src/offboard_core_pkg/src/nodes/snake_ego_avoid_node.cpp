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

class SnakeEgoAvoidNode final : public rclcpp::Node {
public:
  SnakeEgoAvoidNode()
  : Node("snake_ego_avoid_node"), iface_(*this, ctx_) {
    const double rate_hz = declare_parameter<double>("setpoint_rate_hz", 20.0);
    const bool auto_start = declare_parameter<bool>("auto_start", true);
    const double warmup_s = declare_parameter<double>("presetpoint_duration_s", 2.0);
    const double command_timeout_s = declare_parameter<double>(
      "command_timeout_s", 10.0);
    const double command_retry_s = declare_parameter<double>(
      "command_retry_interval_s", 1.0);
    const double takeoff_height_m = declare_parameter<double>(
      "takeoff.height_m", 0.8);
    const double takeoff_tolerance_m = declare_parameter<double>(
      "takeoff_tolerance_m", 0.12);
    const double takeoff_timeout_s = declare_parameter<double>(
      "takeoff_timeout_s", 30.0);
    const double land_timeout_s = declare_parameter<double>(
      "land_timeout_s", 15.0);
    const double land_retry_s = declare_parameter<double>(
      "land_retry_interval_s", 1.0);

    if (rate_hz < 2.0) {
      throw std::invalid_argument("setpoint_rate_hz must be at least 2 Hz");
    }

    const auto goal_topic = declare_parameter<std::string>(
      "ego.goal_topic", "/simple_2d_planner/goal");
    const auto grid_topic = declare_parameter<std::string>(
      "ego.occupancy_grid_topic", "/ego_2d_planner/occupancy_grid");
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

    subscribeEgoInputs();
    subscribeOccupancyGrid(grid_topic);

    SnakeEgoAvoidTask::Config cfg;
    configureSnake(cfg);
    configureEgo(cfg);
    cfg.trigger_distance_m = declare_parameter<double>(
      "avoidance.trigger_distance_m", 0.01);
    cfg.occupancy_timeout_s = declare_parameter<double>(
      "avoidance.occupancy_timeout_s", 0.5);
    cfg.avoidance_timeout_s = declare_parameter<double>(
      "avoidance.timeout_s", 30.0);
    cfg.occupied_threshold = declare_parameter<int>(
      "avoidance.occupied_threshold", 50);
    cfg.landing_timeout_s = land_timeout_s;
    cfg.landing_retry_interval_s = land_retry_s;

    scheduler_.add(std::make_unique<PresetpointTask>(warmup_s));
    scheduler_.add(std::make_unique<SetOffboardTask>(
      command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<ArmTask>(
      command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<TakeoffTask>(
      takeoff_height_m, takeoff_tolerance_m, takeoff_timeout_s));
    scheduler_.add(std::make_unique<SnakeEgoAvoidTask>(
      get_logger(), get_clock(), goal_pub_, cfg));
    scheduler_.add(std::make_unique<LandTask>(
      land_timeout_s, land_retry_s));
    scheduler_.reset();

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this, auto_start] {
        const auto current = now();
        const double dt_s = std::clamp(
          (current - last_tick_).seconds(), 0.0, 0.2);
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

  void configureSnake(SnakeEgoAvoidTask::Config &cfg) {
    const auto first_axis = declare_parameter<std::string>(
      "snake.first_axis", "y_first");
    if (first_axis == "x_first" || first_axis == "X_FIRST" || first_axis == "x") {
      cfg.snake.first_axis = SnakeGridTask::FirstAxis::X_FIRST;
    } else if (first_axis == "y_first" || first_axis == "Y_FIRST" || first_axis == "y") {
      cfg.snake.first_axis = SnakeGridTask::FirstAxis::Y_FIRST;
    } else {
      throw std::invalid_argument(
        "snake.first_axis must be x_first or y_first");
    }
    cfg.snake.stop_mode = SnakeGridTask::StopMode::LINE_END_ONLY;
    cfg.snake.x_cells = declare_parameter<int>("snake.x_cells", 3);
    cfg.snake.y_cells = declare_parameter<int>("snake.y_cells", 3);
    cfg.snake.cell_size = declare_parameter<double>("snake.cell_size_m", 0.8);
    cfg.snake.hover_s = declare_parameter<double>("snake.hover_s", 0.8);
    cfg.snake.max_step_m = declare_parameter<double>("snake.max_step_m", 0.03);
    cfg.snake.arrive_xy_m = declare_parameter<double>("snake.arrive_xy_m", 0.12);
    cfg.snake.arrive_z_m = declare_parameter<double>("snake.arrive_z_m", 0.15);
    cfg.snake.include_start_cell = declare_parameter<bool>(
      "snake.include_start_cell", true);
  }

  void configureEgo(SnakeEgoAvoidTask::Config &cfg) {
    cfg.ego.task_name = "EGO_AVOID";
    cfg.ego.goal_frame = declare_parameter<std::string>(
      "ego.goal_frame", "lidar");
    cfg.ego.planner.use_velocity_ff = declare_parameter<bool>(
      "ego.use_velocity_ff", true);
    cfg.ego.planner.use_acceleration_ff = declare_parameter<bool>(
      "ego.use_acceleration_ff", false);
    cfg.ego.planner.vel_ff_scale = declare_parameter<double>(
      "ego.vel_ff_scale", 0.5);
    cfg.ego.planner.acc_ff_scale = declare_parameter<double>(
      "ego.acc_ff_scale", 0.0);
    // FAST-LIO lidar odometry and MAVROS local position are aligned in ENU.
    cfg.ego.planner.x_sign = declare_parameter<double>("ego.x_sign", 1.0);
    cfg.ego.planner.y_sign = declare_parameter<double>("ego.y_sign", 1.0);
    cfg.ego.planner.swap_xy = declare_parameter<bool>("ego.swap_xy", false);
    cfg.ego.planner.yaw_align_rad = declare_parameter<double>(
      "ego.yaw_align_rad", 0.0);
  }

  void subscribeEgoInputs() {
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
  }

  void subscribeOccupancyGrid(const std::string &topic) {
    occupancy_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        ctx_.occupancy_grid_resolution = msg->info.resolution;
        ctx_.occupancy_grid_origin_x = msg->info.origin.position.x;
        ctx_.occupancy_grid_origin_y = msg->info.origin.position.y;
        ctx_.occupancy_grid_width = msg->info.width;
        ctx_.occupancy_grid_height = msg->info.height;
        ctx_.occupancy_grid_data = msg->data;
        ctx_.occupancy_grid_stamp_us = stampUs(msg->header.stamp);
        if (ctx_.occupancy_grid_stamp_us == 0) {
          ctx_.occupancy_grid_stamp_us = static_cast<std::uint64_t>(
            now().nanoseconds() / 1000ULL);
        }
        ctx_.occupancy_grid_valid =
          ctx_.occupancy_grid_data.size() ==
          static_cast<std::size_t>(msg->info.width) * msg->info.height;
      });
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
  rclcpp::spin(std::make_shared<offboard_core_pkg::SnakeEgoAvoidNode>());
  rclcpp::shutdown();
  return 0;
}
