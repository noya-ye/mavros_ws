#include <cmath>
#include <functional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace offboard_core_pkg {

class LidarToPx4Bridge final : public rclcpp::Node {
public:
  LidarToPx4Bridge() : Node("lidar_to_px4_bridge") {
    const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "/fastlio2/lio_odom");
    const auto vision_pose_topic =
        declare_parameter<std::string>("vision_pose_topic", "/mavros/vision_pose/pose");
    const auto frame_id = declare_parameter<std::string>("frame_id", "map");
    jump_threshold_m_ = declare_parameter<double>("jump_threshold_m", 0.1);
    if (!std::isfinite(jump_threshold_m_) || jump_threshold_m_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid jump_threshold_m %.3f; using 0.1 m", jump_threshold_m_);
      jump_threshold_m_ = 0.1;
    }

    frame_id_ = frame_id;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        std::bind(&LidarToPx4Bridge::odomCallback, this, std::placeholders::_1));
    vision_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(vision_pose_topic, 10);

    RCLCPP_INFO(get_logger(), "Bridging %s to %s in frame %s", odometry_topic.c_str(),
                vision_pose_topic.c_str(), frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "Rejecting odometry position jumps greater than %.3f m",
                jump_threshold_m_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto &position = msg->pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Rejecting odometry with non-finite position");
      return;
    }

    if (has_last_position_) {
      const double dx = position.x - last_position_x_;
      const double dy = position.y - last_position_y_;
      const double dz = position.z - last_position_z_;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance > jump_threshold_m_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejecting lio_odom position jump of %.3f m (threshold %.3f m)", distance,
            jump_threshold_m_);
        return;
      }
    }

    geometry_msgs::msg::PoseStamped vision_pose;
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = frame_id_;
    vision_pose.pose = msg->pose.pose;
    vision_pose_pub_->publish(vision_pose);

    last_position_x_ = position.x;
    last_position_y_ = position.y;
    last_position_z_ = position.z;
    has_last_position_ = true;
  }

  std::string frame_id_;
  double jump_threshold_m_{0.1};
  bool has_last_position_{false};
  double last_position_x_{0.0};
  double last_position_y_{0.0};
  double last_position_z_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
};

}  // namespace offboard_core_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_core_pkg::LidarToPx4Bridge>());
  rclcpp::shutdown();
  return 0;
}
