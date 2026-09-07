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

    frame_id_ = frame_id;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        std::bind(&LidarToPx4Bridge::odomCallback, this, std::placeholders::_1));
    vision_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(vision_pose_topic, 10);

    RCLCPP_INFO(get_logger(), "Bridging %s to %s in frame %s", odometry_topic.c_str(),
                vision_pose_topic.c_str(), frame_id_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped vision_pose;
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = frame_id_;
    vision_pose.pose = msg->pose.pose;
    vision_pose_pub_->publish(vision_pose);
  }

  std::string frame_id_;
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
