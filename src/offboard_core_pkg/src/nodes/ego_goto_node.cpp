#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include "offboard_core_pkg/mavros_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"

namespace offboard_core_pkg {
class EgoGotoNode final : public rclcpp::Node {
public:
  EgoGotoNode() : Node("ego_goto_node"), iface_(*this,ctx_) {
    const auto rate=declare_parameter<double>("setpoint_rate_hz",20.0); const auto auto_start=declare_parameter<bool>("auto_start",true);
    const auto pub_topic=declare_parameter<std::string>("ego.goal_topic","/simple_2d_planner/goal");
    rclcpp::QoS qos(10);
    goal_pub_=create_publisher<geometry_msgs::msg::PoseStamped>(pub_topic,qos);
    ego_cmd_sub_=create_subscription<quadrotor_msgs::msg::PositionCommand>("/position_cmd",rclcpp::SensorDataQoS(),[this](quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg){
      ctx_.ego_cmd_position={msg->position.x,msg->position.y,msg->position.z}; ctx_.ego_cmd_velocity={msg->velocity.x,msg->velocity.y,msg->velocity.z}; ctx_.ego_cmd_acceleration={msg->acceleration.x,msg->acceleration.y,msg->acceleration.z}; ctx_.ego_cmd_yaw=msg->yaw; ctx_.ego_cmd_valid=true; ctx_.ego_cmd_stamp_us=stampUs(msg->header.stamp); });
    ego_odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/fastlio2/lio_odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
      ctx_.ego_odom_position={msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z}; ctx_.ego_odom_velocity={msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z}; ctx_.ego_odom_valid=true; ctx_.ego_odom_stamp_us=stampUs(msg->header.stamp); });
    EgoGotoTask::Config cfg; cfg.x_rel=declare_parameter<double>("goal.x_rel_m",2.0); cfg.y_rel=declare_parameter<double>("goal.y_rel_m",0.0); cfg.height_m=declare_parameter<double>("goal.height_m",1.5); cfg.goal_frame=declare_parameter<std::string>("ego.goal_frame","camera_init"); cfg.planner.use_velocity_ff=declare_parameter<bool>("ego.use_velocity_ff",true); cfg.planner.use_acceleration_ff=declare_parameter<bool>("ego.use_acceleration_ff",false); cfg.planner.vel_ff_scale=declare_parameter<double>("ego.vel_ff_scale",0.5); cfg.planner.acc_ff_scale=declare_parameter<double>("ego.acc_ff_scale",0.0);
    scheduler_.add(std::make_unique<PresetpointTask>(2.0)); scheduler_.add(std::make_unique<SetOffboardTask>()); scheduler_.add(std::make_unique<ArmTask>()); scheduler_.add(std::make_unique<TakeoffTask>(cfg.height_m,0.15)); scheduler_.add(std::make_unique<EgoGotoTask>(get_logger(),get_clock(),goal_pub_,cfg)); scheduler_.add(std::make_unique<LandTask>()); scheduler_.reset();
    last_=now(); timer_=create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0/rate)),[this,auto_start]{auto n=now();auto dt=std::clamp((n-last_).seconds(),0.0,0.2);last_=n;if(auto_start&&!scheduler_.done()&&!scheduler_.failed())scheduler_.tick(ctx_,iface_,dt);iface_.publishSetpoint();});
  }
private:
  static std::uint64_t stampUs(const builtin_interfaces::msg::Time &t){return static_cast<std::uint64_t>(t.sec)*1000000ULL+t.nanosec/1000ULL;}
  Context ctx_; MavrosIface iface_; Scheduler scheduler_; rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_; rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr ego_cmd_sub_; rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_; rclcpp::TimerBase::SharedPtr timer_; rclcpp::Time last_{0,0,RCL_ROS_TIME};
};
}
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<offboard_core_pkg::EgoGotoNode>());rclcpp::shutdown();return 0;}
