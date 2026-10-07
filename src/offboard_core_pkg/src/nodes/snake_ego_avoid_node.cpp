#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"

namespace offboard_core_pkg {

class SnakeEgoAvoidNode final : public rclcpp::Node {
public:
  SnakeEgoAvoidNode()
#ifdef ALIGN_DROP_SNAKE_EGO_NODE
  : Node("align_drop_snake_ego_node"), iface_(*this, ctx_) {
#else
  : Node("snake_ego_avoid_node"), iface_(*this, ctx_) {
#endif
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
    const auto yolo_topic = declare_parameter<std::string>(
      "yolo.detections_topic", "/yolo/detections");

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

    AlignDropSnakeEgoTask::Config cfg;
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

    const auto contour_topic = declare_parameter<std::string>(
      "align_down.contour_topic", "/target/contour_center");
    const auto circle_topic = declare_parameter<std::string>(
      "align_down.circle_topic", "/target/circle_center");
    cfg.align_pixels_per_meter = declare_parameter<double>(
      "align_down.pixels_per_meter", 100.0);
    cfg.align_stable_frames = declare_parameter<int>(
      "align_down.stable_frames", 5);
    cfg.align_arrive_distance_m = declare_parameter<double>(
      "align_down.arrive_distance_m", 0.1);
    cfg.align_max_step_m = declare_parameter<double>(
      "align_down.max_step_m", 0.10);
    cfg.align_timeout_s = declare_parameter<double>("align_down.timeout_s", 10.0);
    cfg.contour_timeout_s = declare_parameter<double>(
      "align_down.detection_timeout_s", 0.5);
    cfg.align_loss_timeout_s = declare_parameter<double>(
      "align_down.loss_timeout_s", 1.0);
    cfg.align_loss_retry_cooldown_s = declare_parameter<double>(
      "align_down.loss_retry_cooldown_s", 3.0);
    cfg.align_trigger_cooldown_s = declare_parameter<double>(
      "align_down.trigger_cooldown_s", 2.0);
    cfg.align_retrigger_radius_m = declare_parameter<double>(
      "align_down.retrigger_radius_m", 0.55);
    cfg.red_cross_enabled = declare_parameter<bool>(
      "red_cross_align.enabled", false);
    const auto red_cross_topic = declare_parameter<std::string>(
      "red_cross_align.topic", "/target/red_cross_center");
    cfg.red_cross_pixels_per_meter = declare_parameter<double>(
      "red_cross_align.pixels_per_meter", 100.0);
    cfg.red_cross_stable_frames = declare_parameter<int>(
      "red_cross_align.stable_frames", 5);
    cfg.red_cross_arrive_distance_m = declare_parameter<double>(
      "red_cross_align.arrive_distance_m", 0.1);
    cfg.red_cross_max_step_m = declare_parameter<double>(
      "red_cross_align.max_step_m", 0.10);
    cfg.red_cross_timeout_s = declare_parameter<double>(
      "red_cross_align.timeout_s", 10.0);
    cfg.drop_height_m = declare_parameter<double>("down_drop.land_height_m", 0.1);
    for (std::size_t i = 0; i < cfg.drop_targets.size(); ++i) {
      const auto prefix = "down_drop.targets.id" + std::to_string(i);
      cfg.drop_targets[i].id = declare_parameter<int>(
        prefix + ".id", static_cast<int>(i));
      cfg.drop_targets[i].offset_x = declare_parameter<double>(
        prefix + ".offset_x_m", 0.0);
      cfg.drop_targets[i].offset_y = declare_parameter<double>(
        prefix + ".offset_y_m", 0.0);
    }
    cfg.drop_serial_device = declare_parameter<std::string>(
      "down_drop.serial_device", "/dev/ttyUSB0");
    const auto drop_baud_rate = declare_parameter<int>("down_drop.baud_rate", 115200);
    if (drop_baud_rate <= 0 ||
        static_cast<std::uint64_t>(drop_baud_rate) >
          std::numeric_limits<unsigned int>::max()) {
      throw std::invalid_argument("down_drop.baud_rate is invalid");
    }
    cfg.drop_serial_baud_rate = static_cast<unsigned int>(drop_baud_rate);
    if (!std::isfinite(cfg.align_max_step_m) || cfg.align_max_step_m <= 0.0) {
      throw std::invalid_argument("align_down.max_step_m must be positive");
    }
    if (!std::isfinite(cfg.align_timeout_s) || cfg.align_timeout_s <= 0.0) {
      throw std::invalid_argument("align_down.timeout_s must be positive");
    }
    if (!std::isfinite(cfg.align_loss_timeout_s) || cfg.align_loss_timeout_s <= 0.0 ||
        !std::isfinite(cfg.align_loss_retry_cooldown_s) ||
        cfg.align_loss_retry_cooldown_s < 0.0 ||
        !std::isfinite(cfg.align_trigger_cooldown_s) ||
        cfg.align_trigger_cooldown_s < 0.0) {
      throw std::invalid_argument("align_down loss timeout/cooldown parameters are invalid");
    }
    if (!std::isfinite(cfg.align_retrigger_radius_m) ||
        cfg.align_retrigger_radius_m < 0.0) {
      throw std::invalid_argument(
        "align_down.retrigger_radius_m must be non-negative");
    }
    if (cfg.red_cross_enabled &&
        (!std::isfinite(cfg.red_cross_pixels_per_meter) ||
         cfg.red_cross_pixels_per_meter <= 0.0 ||
         cfg.red_cross_stable_frames <= 0 ||
         !std::isfinite(cfg.red_cross_arrive_distance_m) ||
         cfg.red_cross_arrive_distance_m <= 0.0 ||
         !std::isfinite(cfg.red_cross_max_step_m) ||
         cfg.red_cross_max_step_m <= 0.0 ||
         !std::isfinite(cfg.red_cross_timeout_s) ||
         cfg.red_cross_timeout_s <= 0.0)) {
      throw std::invalid_argument("red_cross_align parameters are invalid");
    }
    const bool invalid_drop_target = std::any_of(
      cfg.drop_targets.begin(), cfg.drop_targets.end(),
      [](const DownDropTask::obj_id &target) {
        return !std::isfinite(target.offset_x) ||
               !std::isfinite(target.offset_y);
      });
    if (!std::isfinite(cfg.drop_height_m) || invalid_drop_target ||
        cfg.drop_serial_device.empty()) {
      throw std::invalid_argument("down_drop parameters are invalid");
    }
    circle_sub_ = create_subscription<geometry_msgs::msg::Point>(
      circle_topic, 10, [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {
        ctx_.down_circle_offset_px = {msg->x, msg->y, 0.0};
        ctx_.down_circle_stamp = std::chrono::steady_clock::now();
        ++ctx_.down_circle_seq;
      });
    contour_sub_ = create_subscription<geometry_msgs::msg::Point>(
      contour_topic, 10, [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {
        ctx_.down_contour_offset_px = {msg->x, msg->y, 0.0};
        ctx_.down_contour_stamp = std::chrono::steady_clock::now();
        ++ctx_.down_contour_seq;
      });
    if (cfg.red_cross_enabled) {
      red_cross_sub_ = create_subscription<geometry_msgs::msg::Point>(
        red_cross_topic, 10,
        [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {
          ctx_.red_cross_offset_px = {msg->x, msg->y, 0.0};
          ctx_.red_cross_stamp = std::chrono::steady_clock::now();
          ++ctx_.red_cross_seq;
        });
    }
    yolo_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      yolo_topic, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() % 2 != 0) {
          RCLCPP_WARN(get_logger(), "Ignoring malformed YOLO detection array");
          return;
        }

        std::vector<YoloDetection> detections;
        detections.reserve(msg->data.size() / 2);
        for (std::size_t i = 0; i < msg->data.size(); i += 2) {
          const auto class_id = msg->data[i];
          const auto confidence = msg->data[i + 1];
          if (!std::isfinite(class_id) || !std::isfinite(confidence) ||
              class_id < 0.0F || std::floor(class_id) != class_id ||
              static_cast<double>(class_id) >
                static_cast<double>(std::numeric_limits<std::int32_t>::max()) ||
              confidence < 0.0F || confidence > 1.0F) {
            RCLCPP_WARN(get_logger(), "Ignoring malformed YOLO detection pair");
            return;
          }
          detections.push_back(
            {static_cast<std::int32_t>(class_id), confidence});
        }

        ctx_.yolo_detections = std::move(detections);
        ctx_.yolo_detections_stamp = std::chrono::steady_clock::now();
        ++ctx_.yolo_detections_seq;
      });

    scheduler_.add(std::make_unique<PresetpointTask>(warmup_s));
    scheduler_.add(std::make_unique<SetOffboardTask>(
      command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<ArmTask>(
      command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<TakeoffTask>(
      takeoff_height_m, takeoff_tolerance_m, takeoff_timeout_s));
    scheduler_.add(std::make_unique<AlignDropSnakeEgoTask>(
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
        if (ctx_.home_initialized) {
          ctx_.yaw_setpoint_enu = ctx_.home_yaw_enu;
        }
        iface_.publishSetpoint();
      });
  }

private:
  static std::uint64_t stampUs(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<std::uint64_t>(stamp.sec) * 1000000ULL +
           stamp.nanosec / 1000ULL;
  }

  void configureSnake(AlignDropSnakeEgoTask::Config &cfg) {
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

  void configureEgo(AlignDropSnakeEgoTask::Config &cfg) {
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
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr circle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr contour_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr red_cross_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr ego_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr yolo_sub_;
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
