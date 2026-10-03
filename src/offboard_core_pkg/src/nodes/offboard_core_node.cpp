#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"

namespace offboard_core_pkg {
class OffboardCoreNode final : public rclcpp::Node {
public:
  OffboardCoreNode() : Node("offboard_core_node"), iface_(*this, ctx_) {
    const auto rate_hz = declare_parameter<double>("setpoint_rate_hz", 20.0);
    const auto presetpoint_s = declare_parameter<double>("presetpoint_duration_s", 2.0);
    const auto command_timeout_s = declare_parameter<double>("command_timeout_s", 10.0);
    const auto command_retry_s = declare_parameter<double>("command_retry_interval_s", 1.0);
    const auto takeoff_height_m = declare_parameter<double>("takeoff_height_m", 1.2);
    const auto takeoff_tolerance_m = declare_parameter<double>("takeoff_tolerance_m", 0.20);
    const auto takeoff_timeout_s = declare_parameter<double>("takeoff_timeout_s", 30.0);
    const auto hover_s = declare_parameter<double>("hover_duration_s", 1.0);
    const auto align_enabled = declare_parameter<bool>("align_down.enabled", true);
    const auto pixels_per_meter = declare_parameter<double>("align_down.pixels_per_meter", 1000.0);
    const auto stable_frames = declare_parameter<int>("align_down.stable_frames",3);
    const auto arrive_distance_m = declare_parameter<double>("align_down.arrive_distance_m", 0.05);
    const auto max_step_m = declare_parameter<double>("align_down.max_step_m", 0.10);
    const auto circle_topic = declare_parameter<std::string>(
      "align_down.circle_topic", "/target/circle_center");
    const auto contour_topic = declare_parameter<std::string>(
      "align_down.contour_topic", "/target/contour_center");
    const auto yolo_topic = declare_parameter<std::string>(
      "yolo.detections_topic", "/yolo/detections");
    const auto land_timeout_s = declare_parameter<double>("land_timeout_s", 15.0);
    const auto auto_start = declare_parameter<bool>("auto_start", true);
    if (rate_hz < 2.0) throw std::invalid_argument("setpoint_rate_hz must be at least 2 Hz");
    if (align_enabled) {
      if (!std::isfinite(pixels_per_meter) || pixels_per_meter <= 0.0 ||
          stable_frames <= 0 || !std::isfinite(arrive_distance_m) ||
          arrive_distance_m <= 0.0 || !std::isfinite(max_step_m) ||
          max_step_m <= 0.0) {
        throw std::invalid_argument("align_down parameters are invalid");
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

    scheduler_.add(std::make_unique<PresetpointTask>(presetpoint_s));
    scheduler_.add(std::make_unique<SetOffboardTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<ArmTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<TakeoffTask>(takeoff_height_m, takeoff_tolerance_m, takeoff_timeout_s));
    scheduler_.add(std::make_unique<HoverTask>(hover_s));
    if (align_enabled) {
      scheduler_.add(std::make_unique<AlignDownTask>(
        pixels_per_meter, stable_frames, arrive_distance_m, max_step_m));
    }
    scheduler_.add(std::make_unique<LandTask>(land_timeout_s, command_retry_s));
    scheduler_.reset();

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    last_tick_ = now();
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this, auto_start] {
      const auto current = now();
      const auto dt_s = (current - last_tick_).seconds();
      last_tick_ = current;
      if (auto_start && !scheduler_.done() && !scheduler_.failed()) scheduler_.tick(ctx_, iface_, dt_s);
      iface_.publishSetpoint();
    });
  }

private:
  Context ctx_;
  MavrosIface iface_;
  Scheduler scheduler_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr circle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr contour_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr yolo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
};
}  // namespace offboard_core_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_core_pkg::OffboardCoreNode>());
  rclcpp::shutdown();
  return 0;
}
