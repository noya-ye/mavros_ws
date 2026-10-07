#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace offboard_core_pkg {

class LidarToPx4Bridge final : public rclcpp::Node {
public:
  LidarToPx4Bridge()
  : Node("lidar_to_px4_bridge")
  {
    // ============================================================
    // ROS 2 parameters
    // ============================================================

    const auto odometry_topic =
        declare_parameter<std::string>(
            "odometry_topic",
            "/fastlio2/lio_odom");

    const auto vision_pose_topic =
        declare_parameter<std::string>(
            "vision_pose_topic",
            "/mavros/vision_pose/pose");

    frame_id_ =
        declare_parameter<std::string>(
            "frame_id",
            "map");

    // 最大允许实际运动速度。
    //
    // 正常帧允许的位移：
    //
    //   allowed_distance =
    //       max_velocity_mps * dt + jump_margin_m
    //
    max_velocity_mps_ =
        declare_parameter<double>(
            "max_velocity_mps",
            3.0);

    // 给 FAST-LIO 的正常定位抖动留出的额外余量。
    jump_margin_m_ =
        declare_parameter<double>(
            "jump_margin_m",
            0.10);

    // 连续多少个异常区域中的样本自身保持连续后，
    // 认为 FAST-LIO 已经稳定到新的坐标分支。
    resync_consecutive_frames_ =
        declare_parameter<int>(
            "resync_consecutive_frames",
            5);

    // dt 上限。
    //
    // 防止 FAST-LIO 长时间没有消息以后，
    // 因为 dt 特别大而使 allowed_distance 变得特别大。
    max_dt_s_ =
        declare_parameter<double>(
            "max_dt_s",
            0.50);

    // ============================================================
    // Parameter validation
    // ============================================================

    if (!std::isfinite(max_velocity_mps_) ||
        max_velocity_mps_ <= 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "Invalid max_velocity_mps %.3f; using 3.0 m/s",
          max_velocity_mps_);

      max_velocity_mps_ = 3.0;
    }

    if (!std::isfinite(jump_margin_m_) ||
        jump_margin_m_ < 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "Invalid jump_margin_m %.3f; using 0.10 m",
          jump_margin_m_);

      jump_margin_m_ = 0.10;
    }

    if (resync_consecutive_frames_ < 2)
    {
      RCLCPP_WARN(
          get_logger(),
          "Invalid resync_consecutive_frames %d; using 5",
          resync_consecutive_frames_);

      resync_consecutive_frames_ = 5;
    }

    if (!std::isfinite(max_dt_s_) ||
        max_dt_s_ <= 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "Invalid max_dt_s %.3f; using 0.50 s",
          max_dt_s_);

      max_dt_s_ = 0.50;
    }

    // ============================================================
    // ROS interface
    // ============================================================

    odom_sub_ =
        create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic,
            rclcpp::SensorDataQoS(),
            std::bind(
                &LidarToPx4Bridge::odomCallback,
                this,
                std::placeholders::_1));

    vision_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(
            vision_pose_topic,
            10);

    // ============================================================
    // Startup log
    // ============================================================

    RCLCPP_INFO(
        get_logger(),
        "Bridging %s -> %s, frame=%s",
        odometry_topic.c_str(),
        vision_pose_topic.c_str(),
        frame_id_.c_str());

    RCLCPP_INFO(
        get_logger(),
        "Position filter: max_velocity=%.3f m/s, "
        "margin=%.3f m, max_dt=%.3f s",
        max_velocity_mps_,
        jump_margin_m_,
        max_dt_s_);

    RCLCPP_INFO(
        get_logger(),
        "Resync after %d consecutive stable abnormal samples",
        resync_consecutive_frames_);
  }

private:
  // ==============================================================
  // Utility
  // ==============================================================

  static double distance3D(
      double x1,
      double y1,
      double z1,
      double x2,
      double y2,
      double z2)
  {
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    const double dz = z1 - z2;

    return std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz);
  }

  double allowedDistance(double dt) const
  {
    // 避免掉帧以后 dt 非常大，
    // 导致速度门限失去意义。
    const double effective_dt =
        std::clamp(
            dt,
            0.0,
            max_dt_s_);

    return
        max_velocity_mps_ * effective_dt +
        jump_margin_m_;
  }

  void resetResyncState()
  {
    resync_active_ = false;
    resync_streak_ = 0;
    has_resync_previous_ = false;
  }

  // ==============================================================
  // Publish
  // ==============================================================

  void publishVisionPose(
      const nav_msgs::msg::Odometry::SharedPtr &msg)
  {
    geometry_msgs::msg::PoseStamped vision_pose;

    vision_pose.header.stamp =
        msg->header.stamp;

    vision_pose.header.frame_id =
        frame_id_;

    // orientation 暂时原样传递 FAST-LIO 的姿态。
    vision_pose.pose.orientation =
        msg->pose.pose.orientation;

    // 对位置加入软重同步 offset。
    //
    // 正常情况下 offset = 0。
    //
    // 如果 FAST-LIO 整个坐标突然跳变，
    // offset 用来保证 MAVROS 看到的位置仍然连续。
    vision_pose.pose.position.x =
        msg->pose.pose.position.x +
        output_offset_x_;

    vision_pose.pose.position.y =
        msg->pose.pose.position.y +
        output_offset_y_;

    vision_pose.pose.position.z =
        msg->pose.pose.position.z +
        output_offset_z_;

    vision_pose_pub_->publish(vision_pose);

    last_published_x_ =
        vision_pose.pose.position.x;

    last_published_y_ =
        vision_pose.pose.position.y;

    last_published_z_ =
        vision_pose.pose.position.z;

    has_last_published_ = true;
  }

  // ==============================================================
  // Accept normal sample
  // ==============================================================

  void acceptSample(
      const nav_msgs::msg::Odometry::SharedPtr &msg,
      const rclcpp::Time &stamp)
  {
    const auto &position =
        msg->pose.pose.position;

    publishVisionPose(msg);

    last_raw_x_ = position.x;
    last_raw_y_ = position.y;
    last_raw_z_ = position.z;

    last_accepted_stamp_ = stamp;

    has_last_accepted_ = true;

    // 一旦重新回到正常轨迹，
    // 清除之前的异常候选状态。
    resetResyncState();
  }

  // ==============================================================
  // Handle abnormal sample
  // ==============================================================

  void handleAbnormalSample(
      const nav_msgs::msg::Odometry::SharedPtr &msg,
      const rclcpp::Time &stamp,
      double trusted_distance,
      double trusted_dt,
      double trusted_allowed)
  {
    const auto &position =
        msg->pose.pose.position;

    // ------------------------------------------------------------
    // 第一帧异常
    // ------------------------------------------------------------

    if (!resync_active_)
    {
      resync_active_ = true;
      resync_streak_ = 1;

      resync_previous_x_ = position.x;
      resync_previous_y_ = position.y;
      resync_previous_z_ = position.z;

      resync_previous_stamp_ = stamp;

      has_resync_previous_ = true;

      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Rejecting lio_odom: "
          "distance=%.3f m dt=%.3f s "
          "estimated_speed=%.3f m/s "
          "allowed=%.3f m; "
          "starting resync candidate 1/%d",
          trusted_distance,
          trusted_dt,
          trusted_dt > 0.0
              ? trusted_distance / trusted_dt
              : 0.0,
          trusted_allowed,
          resync_consecutive_frames_);

      return;
    }

    // ------------------------------------------------------------
    // 已经处在异常状态
    //
    // 此时不再仅仅比较：
    //
    //   current <-> 最后的正常位置
    //
    // 同时比较：
    //
    //   current <-> 上一帧异常样本
    //
    // 如果这些异常位置彼此是连续的，
    // 说明 FAST-LIO 很可能已经稳定到了新的坐标分支。
    // ------------------------------------------------------------

    if (!has_resync_previous_)
    {
      resync_previous_x_ = position.x;
      resync_previous_y_ = position.y;
      resync_previous_z_ = position.z;

      resync_previous_stamp_ = stamp;

      has_resync_previous_ = true;

      resync_streak_ = 1;

      return;
    }

    const double candidate_dt =
        (stamp - resync_previous_stamp_).seconds();

    bool candidate_continuous = false;

    double candidate_distance = 0.0;
    double candidate_allowed = 0.0;

    if (candidate_dt > 0.0 &&
        std::isfinite(candidate_dt))
    {
      candidate_distance =
          distance3D(
              position.x,
              position.y,
              position.z,
              resync_previous_x_,
              resync_previous_y_,
              resync_previous_z_);

      candidate_allowed =
          allowedDistance(candidate_dt);

      candidate_continuous =
          candidate_distance <= candidate_allowed;
    }

    // ------------------------------------------------------------
    // 异常区域内部也是连续运动
    // ------------------------------------------------------------

    if (candidate_continuous)
    {
      ++resync_streak_;
    }
    else
    {
      // 新位置本身仍然乱跳。
      //
      // 不能认为已经稳定，
      // 重新从当前样本开始计数。
      resync_streak_ = 1;

      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Resync candidate unstable: "
          "distance=%.3f m dt=%.3f s "
          "allowed=%.3f m; restarting streak",
          candidate_distance,
          candidate_dt,
          candidate_allowed);
    }

    // 无论这一帧是否连续，
    // 下一帧都从当前异常样本继续观察。
    resync_previous_x_ = position.x;
    resync_previous_y_ = position.y;
    resync_previous_z_ = position.z;

    resync_previous_stamp_ = stamp;

    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "lio_odom still outside trusted trajectory: "
        "trusted_distance=%.3f m, "
        "candidate_distance=%.3f m, "
        "resync_streak=%d/%d",
        trusted_distance,
        candidate_distance,
        resync_streak_,
        resync_consecutive_frames_);

    // ------------------------------------------------------------
    // 尚未稳定足够多帧
    // ------------------------------------------------------------

    if (resync_streak_ <
        resync_consecutive_frames_)
    {
      return;
    }

    // ============================================================
    // SOFT RESYNC
    // ============================================================
    //
    // 举例：
    //
    // MAVROS 上一次收到：
    //
    //   x = 5.0
    //
    // FAST-LIO 突然重定位为：
    //
    //   x = 7.0
    //
    // 如果直接恢复发布：
    //
    //   MAVROS:
    //
    //   5.0 -> 7.0
    //
    // 会直接出现 2m 跳变。
    //
    //
    // 这里改成：
    //
    //   offset = 5.0 - 7.0 = -2.0
    //
    // 所以：
    //
    //   output = 7.0 - 2.0 = 5.0
    //
    // MAVROS 看到的位置仍然连续。
    //
    // 后续 FAST-LIO：
    //
    //   7.1 -> output 5.1
    //   7.2 -> output 5.2
    //
    // 因此后续相对运动仍然保留。
    // ============================================================

    if (has_last_published_)
    {
      output_offset_x_ =
          last_published_x_ -
          position.x;

      output_offset_y_ =
          last_published_y_ -
          position.y;

      output_offset_z_ =
          last_published_z_ -
          position.z;
    }
    else
    {
      output_offset_x_ = 0.0;
      output_offset_y_ = 0.0;
      output_offset_z_ = 0.0;
    }

    RCLCPP_WARN(
        get_logger(),
        "FAST-LIO soft resync accepted after %d stable abnormal samples. "
        "New output offset=(%.3f, %.3f, %.3f) m",
        resync_streak_,
        output_offset_x_,
        output_offset_y_,
        output_offset_z_);

    // 当前 FAST-LIO 点成为新的 raw reference。
    last_raw_x_ = position.x;
    last_raw_y_ = position.y;
    last_raw_z_ = position.z;

    last_accepted_stamp_ = stamp;

    has_last_accepted_ = true;

    // 用新的 offset 发布。
    publishVisionPose(msg);

    resetResyncState();
  }

  // ==============================================================
  // Odometry callback
  // ==============================================================

  void odomCallback(
      const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &position =
        msg->pose.pose.position;

    // ------------------------------------------------------------
    // Position sanity check
    // ------------------------------------------------------------

    if (!std::isfinite(position.x) ||
        !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Rejecting odometry with non-finite position");

      return;
    }

    const rclcpp::Time current_stamp(
        msg->header.stamp,
        RCL_ROS_TIME);

    // ------------------------------------------------------------
    // First sample
    // ------------------------------------------------------------

    if (!has_last_accepted_)
    {
      output_offset_x_ = 0.0;
      output_offset_y_ = 0.0;
      output_offset_z_ = 0.0;

      acceptSample(
          msg,
          current_stamp);

      RCLCPP_INFO(
          get_logger(),
          "Accepted first FAST-LIO sample: "
          "(%.3f, %.3f, %.3f)",
          position.x,
          position.y,
          position.z);

      return;
    }

    // ------------------------------------------------------------
    // dt relative to last trusted FAST-LIO sample
    // ------------------------------------------------------------

    const double dt =
        (current_stamp -
         last_accepted_stamp_)
            .seconds();

    if (!std::isfinite(dt) ||
        dt <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Rejecting odometry because timestamp dt=%.6f s "
          "is invalid",
          dt);

      return;
    }

    // ------------------------------------------------------------
    // Movement relative to trusted FAST-LIO raw position
    // ------------------------------------------------------------

    const double distance =
        distance3D(
            position.x,
            position.y,
            position.z,
            last_raw_x_,
            last_raw_y_,
            last_raw_z_);

    const double allowed =
        allowedDistance(dt);

    // ============================================================
    // NORMAL
    // ============================================================

    if (distance <= allowed)
    {
      acceptSample(
          msg,
          current_stamp);

      return;
    }

    // ============================================================
    // ABNORMAL
    // ============================================================

    handleAbnormalSample(
        msg,
        current_stamp,
        distance,
        dt,
        allowed);
  }

  // ==============================================================
  // Parameters
  // ==============================================================

  std::string frame_id_;

  double max_velocity_mps_{2.0};

  double jump_margin_m_{0.10};

  int resync_consecutive_frames_{5};

  double max_dt_s_{0.50};

  // ==============================================================
  // Last accepted FAST-LIO RAW position
  // ==============================================================

  bool has_last_accepted_{false};

  double last_raw_x_{0.0};
  double last_raw_y_{0.0};
  double last_raw_z_{0.0};

  rclcpp::Time last_accepted_stamp_{
      0,
      0,
      RCL_ROS_TIME};

  // ==============================================================
  // Last position actually sent to MAVROS
  // ==============================================================

  bool has_last_published_{false};

  double last_published_x_{0.0};
  double last_published_y_{0.0};
  double last_published_z_{0.0};

  // ==============================================================
  // Soft-resync output offset
  // ==============================================================

  double output_offset_x_{0.0};
  double output_offset_y_{0.0};
  double output_offset_z_{0.0};

  // ==============================================================
  // Resync candidate
  // ==============================================================

  bool resync_active_{false};

  int resync_streak_{0};

  bool has_resync_previous_{false};

  double resync_previous_x_{0.0};
  double resync_previous_y_{0.0};
  double resync_previous_z_{0.0};

  rclcpp::Time resync_previous_stamp_{
      0,
      0,
      RCL_ROS_TIME};

  // ==============================================================
  // ROS
  // ==============================================================

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      vision_pose_pub_;
};

}  // namespace offboard_core_pkg

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
      std::make_shared<
          offboard_core_pkg::LidarToPx4Bridge>());

  rclcpp::shutdown();

  return 0;
}