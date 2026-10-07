#include <cmath>
#include <mutex>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudSelfFilterNode : public rclcpp::Node
{
public:
  CloudSelfFilterNode()
  : Node("cloud_self_filter_node")
  {
    cloud_in_ = this->declare_parameter<std::string>("cloud_in", "/fastlio2/world_cloud");
    cloud_out_ = this->declare_parameter<std::string>("cloud_out", "/cloud_registered_filtered");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/Odometry");

    use_odom_ = this->declare_parameter<bool>("use_odom", true);

    enable_self_filter_ = this->declare_parameter<bool>("enable_self_filter", true);
    self_filter_radius_ = this->declare_parameter<double>("self_filter_radius", 0.60);

    enable_range_filter_ = this->declare_parameter<bool>("enable_range_filter", true);
    min_range_ = this->declare_parameter<double>("min_range", 0.10);
    max_range_ = this->declare_parameter<double>("max_range", 8.00);

    // The input cloud is in the world frame, so use its z value as height
    // above the ground.  Points higher than 2 m are removed by default.
    enable_z_filter_ = this->declare_parameter<bool>("enable_z_filter", true);
    z_relative_to_odom_ = this->declare_parameter<bool>("z_relative_to_odom", false);
    min_z_keep_ = this->declare_parameter<double>("min_z_keep", 0.10);
    max_z_keep_ = this->declare_parameter<double>("max_z_keep", 2.00);

    enable_voxel_filter_ = this->declare_parameter<bool>("enable_voxel_filter", true);
    voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.10);

    enable_radius_filter_ = this->declare_parameter<bool>("enable_radius_filter", true);
    radius_filter_radius_ = this->declare_parameter<double>("radius_filter_radius", 0.25);
    radius_filter_min_neighbors_ = this->declare_parameter<int>("radius_filter_min_neighbors", 2);

    print_debug_ = this->declare_parameter<bool>("print_debug", true);

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    pub_qos.reliable();
    pub_qos.durability_volatile();

    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_out_, pub_qos);

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_,
      rclcpp::SensorDataQoS(),
      std::bind(&CloudSelfFilterNode::cloudCallback, this, std::placeholders::_1));

    if (use_odom_) {
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&CloudSelfFilterNode::odomCallback, this, std::placeholders::_1));
    }

    RCLCPP_WARN(this->get_logger(), "============================================================");
    RCLCPP_WARN(this->get_logger(), "cloud_self_filter_node started");
    RCLCPP_WARN(this->get_logger(), "cloud_in  : %s", cloud_in_.c_str());
    RCLCPP_WARN(this->get_logger(), "cloud_out : %s", cloud_out_.c_str());
    RCLCPP_WARN(this->get_logger(), "odom_topic: %s", odom_topic_.c_str());
    RCLCPP_WARN(this->get_logger(), "self_filter_radius=%.2f", self_filter_radius_);
    RCLCPP_WARN(this->get_logger(), "range=[%.2f, %.2f]", min_range_, max_range_);
    RCLCPP_WARN(this->get_logger(), "voxel_leaf_size=%.2f", voxel_leaf_size_);
    RCLCPP_WARN(this->get_logger(), "radius_filter_radius=%.2f min_neighbors=%d",
      radius_filter_radius_, radius_filter_min_neighbors_);
    RCLCPP_WARN(this->get_logger(), "============================================================");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_z_ = msg->pose.pose.position.z;
    have_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_raw);

    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    bool have_odom_local = false;

    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      ox = odom_x_;
      oy = odom_y_;
      oz = odom_z_;
      have_odom_local = have_odom_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_basic(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_basic->reserve(cloud_raw->size());

    std::size_t removed_nan = 0;
    std::size_t removed_self = 0;
    std::size_t removed_range = 0;
    std::size_t removed_z = 0;

    for (const auto & p : cloud_raw->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        removed_nan++;
        continue;
      }

      double dx = p.x;
      double dy = p.y;
      double dz = p.z;

      if (use_odom_ && have_odom_local) {
        dx = p.x - ox;
        dy = p.y - oy;
        dz = p.z - oz;
      }

      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (enable_self_filter_ && use_odom_ && have_odom_local) {
        if (dist < self_filter_radius_) {
          removed_self++;
          continue;
        }
      }

      if (enable_range_filter_) {
        if (dist < min_range_ || dist > max_range_) {
          removed_range++;
          continue;
        }
      }

      if (enable_z_filter_) {
        const double z_check = z_relative_to_odom_ ? dz : p.z;
        if (z_check < min_z_keep_ || z_check > max_z_keep_) {
          removed_z++;
          continue;
        }
      }

      cloud_basic->push_back(p);
    }

    cloud_basic->width = static_cast<uint32_t>(cloud_basic->size());
    cloud_basic->height = 1;
    cloud_basic->is_dense = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>());

    if (enable_voxel_filter_ && voxel_leaf_size_ > 0.001 && !cloud_basic->empty()) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(cloud_basic);
      voxel.setLeafSize(
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_));
      voxel.filter(*cloud_voxel);
    } else {
      cloud_voxel = cloud_basic;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>());

    if (enable_radius_filter_ &&
        radius_filter_radius_ > 0.001 &&
        radius_filter_min_neighbors_ > 0 &&
        !cloud_voxel->empty())
    {
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
      radius_filter.setInputCloud(cloud_voxel);
      radius_filter.setRadiusSearch(radius_filter_radius_);
      radius_filter.setMinNeighborsInRadius(radius_filter_min_neighbors_);
      radius_filter.filter(*cloud_final);
    } else {
      cloud_final = cloud_voxel;
    }

    cloud_final->width = static_cast<uint32_t>(cloud_final->size());
    cloud_final->height = 1;
    cloud_final->is_dense = true;

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud_final, out_msg);
    out_msg.header = msg->header;
    pub_cloud_->publish(out_msg);

    if (print_debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[CloudSelfFilter] raw=%zu basic=%zu voxel=%zu final=%zu removed_nan=%zu self=%zu range=%zu z=%zu odom=%d pos=(%.2f %.2f %.2f)",
        cloud_raw->size(),
        cloud_basic->size(),
        cloud_voxel->size(),
        cloud_final->size(),
        removed_nan,
        removed_self,
        removed_range,
        removed_z,
        have_odom_local ? 1 : 0,
        ox, oy, oz);
    }

    if (use_odom_ && !have_odom_local) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[CloudSelfFilter] waiting odom: %s", odom_topic_.c_str());
    }
  }

private:
  std::string cloud_in_;
  std::string cloud_out_;
  std::string odom_topic_;

  bool use_odom_{true};

  bool enable_self_filter_{true};
  double self_filter_radius_{0.60};

  bool enable_range_filter_{true};
  double min_range_{0.10};
  double max_range_{8.00};

  bool enable_z_filter_{true};
  bool z_relative_to_odom_{false};
  double min_z_keep_{0.0};
  double max_z_keep_{2.00};

  bool enable_voxel_filter_{true};
  double voxel_leaf_size_{0.10};

  bool enable_radius_filter_{true};
  double radius_filter_radius_{0.22};
  int radius_filter_min_neighbors_{2};

  bool print_debug_{true};

  std::mutex odom_mutex_;
  bool have_odom_{false};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_z_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudSelfFilterNode>());
  rclcpp::shutdown();
  return 0;
}
