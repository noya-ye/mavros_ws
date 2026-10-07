#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "ego_2d_planner_pkg/msg/polynomial2_d.hpp"
#include "ego_2d_planner_pkg/msg/emergency_stop2_d.hpp"
#include "ego_2d_planner_pkg/diff_planner/polynomial_trajectory_2d.hpp"

using ego_2d_planner_pkg::PolynomialTrajectory2D;
using ego_2d_planner_pkg::Vec2;
using ego_2d_planner_pkg::msg::Polynomial2D;
using ego_2d_planner_pkg::msg::EmergencyStop2D;

class TrajServer2DNode : public rclcpp::Node
{
public:
  TrajServer2DNode() : Node("traj_server_2d_node")
  {
    const auto trajectory_topic = declare_parameter<std::string>("polynomial_topic", "/ego_2d_planner/polynomial_2d");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
    const auto command_topic = declare_parameter<std::string>("cmd_topic", "/position_cmd");
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar");
    const double rate = declare_parameter<double>("cmd_rate_hz", 50.0);
    fixed_z_ = declare_parameter<double>("fixed_z", 1.0);
    use_msg_z_ = declare_parameter<bool>("use_msg_z", true);
    max_vel_ = declare_parameter<double>("max_vel", 0.35);
    max_acc_ = declare_parameter<double>("max_acc", 0.8);
    if (!std::isfinite(rate) || rate < 1.0 || rate > 1000.0 ||
        !std::isfinite(fixed_z_) || !std::isfinite(max_vel_) || max_vel_ <= 0 ||
        !std::isfinite(max_acc_) || max_acc_ <= 0)
      throw std::invalid_argument("invalid trajectory server rate, height or limits");
    cmd_.kx[0] = declare_parameter<double>("pos_gain_x", 0.0);
    cmd_.kx[1] = declare_parameter<double>("pos_gain_y", 0.0);
    cmd_.kx[2] = declare_parameter<double>("pos_gain_z", 0.0);
    cmd_.kv[0] = declare_parameter<double>("vel_gain_x", 0.0);
    cmd_.kv[1] = declare_parameter<double>("vel_gain_y", 0.0);
    cmd_.kv[2] = declare_parameter<double>("vel_gain_z", 0.0);
    command_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(command_topic, 50);
    trajectory_sub_ = create_subscription<Polynomial2D>(trajectory_topic, 10,
      std::bind(&TrajServer2DNode::trajectoryCallback, this, std::placeholders::_1));
    stop_sub_ = create_subscription<EmergencyStop2D>("/ego_2d_planner/emergency_stop",
      rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&TrajServer2DNode::stopCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, 20,
      std::bind(&TrajServer2DNode::odomCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
      std::bind(&TrajServer2DNode::commandCallback, this));
    RCLCPP_WARN(get_logger(), "Diff MINCO server: yaw latches first valid odometry and yaw_dot stays zero");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_yaw_) return;
    const auto& q = msg->pose.pose.orientation;
    const double squared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(squared) || squared < 1e-12) return;
    // Formula with squared norm also handles finite non-unit quaternions.
    locked_yaw_ = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                            squared - 2.0 * (q.y*q.y + q.z*q.z));
    have_yaw_ = true;
  }

  void trajectoryCallback(const Polynomial2D::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->header.frame_id != frame_id_) return;
    if (msg->header.stamp.sec < 0 || msg->start_time.sec < 0 ||
        msg->header.stamp.nanosec >= 1000000000u || msg->start_time.nanosec >= 1000000000u) return;
    const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    if ((have_stop_stamp_ && stamp <= stop_stamp_) || (have_traj_stamp_ && stamp <= traj_stamp_)) return;
    const std::size_t count = msg->durations.size();
    if (msg->order != 5 || count == 0 || count > 64 || msg->coeff_x.size() != 6 * count ||
        msg->coeff_y.size() != 6 * count || !std::isfinite(msg->fixed_z)) {
      RCLCPP_WARN(get_logger(), "Rejected malformed Polynomial2D");
      return;
    }
    PolynomialTrajectory2D candidate;
    candidate.pieces.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      candidate.pieces[i].duration = msg->durations[i];
      for (int k = 0; k < 6; ++k)
        candidate.pieces[i].coefficients[k] = {msg->coeff_x[6*i+k], msg->coeff_y[6*i+k]};
    }
    if (!candidate.valid() || candidate.duration() > 600.0 || !candidate.feasible(max_vel_, max_acc_)) {
      RCLCPP_WARN(get_logger(), "Rejected non-finite or dynamically infeasible Polynomial2D");
      return;
    }
    // A disconnected piece must never cause a command jump.
    double junction = 0.0;
    for (std::size_t i = 1; i < count; ++i) {
      junction += candidate.pieces[i-1].duration;
      PolynomialTrajectory2D next;
      next.pieces.push_back(candidate.pieces[i]);
      for (int d = 0; d <= 2; ++d)
        if (ego_2d_planner_pkg::dist(candidate.evaluate(junction, d), next.evaluate(0.0, d)) > 1e-6) {
          RCLCPP_WARN(get_logger(), "Rejected discontinuous Polynomial2D"); return;
        }
    }
    if (ego_2d_planner_pkg::norm(candidate.evaluate(candidate.duration(), 1)) > 1e-6 ||
        ego_2d_planner_pkg::norm(candidate.evaluate(candidate.duration(), 2)) > 1e-6) {
      RCLCPP_WARN(get_logger(), "Rejected Polynomial2D without a stationary terminal state"); return;
    }
    const rclcpp::Time switch_time(msg->start_time);
    const PolynomialTrajectory2D* previous = nullptr;
    rclcpp::Time previous_start = start_time_;
    if (receive_traj_ && !emergency_hold_) previous = &trajectory_;
    if (have_pending_ && switch_time >= pending_start_time_) {
      previous = &pending_trajectory_; previous_start = pending_start_time_;
    }
    if (previous) {
      const double previous_time = std::clamp((switch_time - previous_start).seconds(), 0.0, previous->duration());
      for (int derivative = 0; derivative <= 2; ++derivative)
        if (ego_2d_planner_pkg::dist(previous->evaluate(previous_time, derivative),
                                   candidate.evaluate(0.0, derivative)) > 1e-4) {
          RCLCPP_WARN(get_logger(), "Rejected Polynomial2D with a discontinuous switch state"); return;
        }
    }
    traj_stamp_ = stamp; have_traj_stamp_ = true;
    pending_trajectory_ = std::move(candidate);
    pending_start_time_ = rclcpp::Time(msg->start_time);
    pending_traj_id_ = msg->traj_id;
    pending_z_ = use_msg_z_ ? msg->fixed_z : fixed_z_;
    have_pending_ = true;
  }

  void stopCallback(const EmergencyStop2D::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->header.frame_id != frame_id_) return;
    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) return;
    const auto& p = msg->position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
    const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    if ((have_stop_stamp_ && stamp < stop_stamp_) || (have_traj_stamp_ && stamp < traj_stamp_)) return;
    if (!have_stop_stamp_ || stamp > stop_stamp_) {
      hold_position_ = {p.x, p.y}; hold_z_ = p.z; stop_stamp_ = stamp;
    }
    have_stop_stamp_ = true;
    trajectory_.pieces.clear(); pending_trajectory_.pieces.clear();
    receive_traj_ = have_pending_ = false; emergency_hold_ = true;
    publishCommand(now(), hold_position_, {}, {}, hold_z_);
  }

  void publishCommand(const rclcpp::Time& stamp, const Vec2& p, const Vec2& v, const Vec2& a, double z)
  {
    if (!have_yaw_) return;
    cmd_.header.stamp = stamp; cmd_.header.frame_id = frame_id_;
    cmd_.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd_.trajectory_id = traj_id_;
    cmd_.position.x = p.x; cmd_.position.y = p.y; cmd_.position.z = z;
    cmd_.velocity.x = v.x; cmd_.velocity.y = v.y; cmd_.velocity.z = 0.0;
    cmd_.acceleration.x = a.x; cmd_.acceleration.y = a.y; cmd_.acceleration.z = 0.0;
    cmd_.yaw = locked_yaw_; cmd_.yaw_dot = 0.0;
    command_pub_->publish(cmd_);
  }

  void commandCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto stamp = now();
    // A clock rewind cancels the active plan instead of replaying old motion.
    if (have_command_time_ && stamp < last_command_time_) {
      if (receive_traj_) {
        hold_position_ = trajectory_.evaluate(last_trajectory_time_);
        hold_z_ = trajectory_z_;
      }
      receive_traj_ = have_pending_ = false; emergency_hold_ = true;
      trajectory_.pieces.clear(); pending_trajectory_.pieces.clear();
    }
    last_command_time_ = stamp; have_command_time_ = true;
    if (have_pending_ && stamp >= pending_start_time_) {
      trajectory_ = std::move(pending_trajectory_); start_time_ = pending_start_time_;
      traj_id_ = pending_traj_id_; trajectory_z_ = pending_z_;
      have_pending_ = false; receive_traj_ = true; emergency_hold_ = false;
    }
    if (emergency_hold_) { publishCommand(stamp, hold_position_, {}, {}, hold_z_); return; }
    if (!receive_traj_ && have_pending_) {
      publishCommand(stamp, pending_trajectory_.evaluate(0.0), {}, {}, pending_z_); return;
    }
    if (!receive_traj_ || !have_yaw_) return;
    const double elapsed = (stamp - start_time_).seconds();
    const double time = std::clamp(elapsed, 0.0, trajectory_.duration());
    last_trajectory_time_ = time;
    const bool executing = elapsed >= 0.0 && elapsed < trajectory_.duration();
    publishCommand(stamp, trajectory_.evaluate(time),
      executing ? trajectory_.evaluate(time, 1) : Vec2{},
      executing ? trajectory_.evaluate(time, 2) : Vec2{}, trajectory_z_);
  }

  std::mutex mutex_;
  std::string frame_id_;
  double fixed_z_{1.0}, trajectory_z_{1.0}, max_vel_{0.35}, max_acc_{0.8};
  double locked_yaw_{0.0}, hold_z_{1.0}, last_trajectory_time_{0.0};
  bool use_msg_z_{true}, have_yaw_{false}, receive_traj_{false}, emergency_hold_{false};
  bool have_stop_stamp_{false}, have_traj_stamp_{false}, have_command_time_{false};
  int64_t stop_stamp_{0}, traj_stamp_{0};
  int traj_id_{0};
  Vec2 hold_position_;
  PolynomialTrajectory2D trajectory_;
  PolynomialTrajectory2D pending_trajectory_;
  bool have_pending_{false};
  double pending_z_{1.0};
  int pending_traj_id_{0};
  rclcpp::Time pending_start_time_;
  rclcpp::Time start_time_, last_command_time_;
  quadrotor_msgs::msg::PositionCommand cmd_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<Polynomial2D>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<EmergencyStop2D>::SharedPtr stop_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajServer2DNode>());
  rclcpp::shutdown();
  return 0;
}
