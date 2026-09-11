#pragma once
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/planners/ego_vel_planner.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
namespace offboard_core_pkg {
class EgoGotoTask final : public ITask {
public:
  struct Config { std::string task_name{"EGO_GOTO"}; std::string goal_name{"GOAL"}; std::string goal_frame{"camera_init"}; double x_rel{0}, y_rel{0}, height_m{1.5}, yaw_local{0}; double arrive_xy_m{0.15}, arrive_z_m{0.15}, stable_vxy_mps{0.15}, stable_vz_mps{0.12}, stable_required_s{0.4}, goal_republish_s{1.0}, cmd_guard_s{0.3}; EgoVelPlanner::Config planner; };
  EgoGotoTask(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub, const Config &cfg) : logger_(logger), clock_(clock), goal_pub_(pub), cfg_(cfg), planner_(cfg.planner) {}
  std::string name() const override { return cfg_.task_name; }
  // Set an absolute ENU target for the next onEnter().
  void setTargetEnu(double x, double y, double z);
  void onEnter(Context &ctx, MavrosIface &) override;
  Status tick(Context &ctx, MavrosIface &, double dt_s) override;
  void onExit(Context &ctx, MavrosIface &) override;
private:
  void publishGoal(const Context &ctx); 
  void hold(Context &ctx); 
  void inverseMap(double x, double y, double &out_x, double &out_y) const; 
  rclcpp::Logger logger_; 
  rclcpp::Clock::SharedPtr clock_; 
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_; 
  Config cfg_; 
  EgoVelPlanner planner_; 
  bool started_{false}; 
  double elapsed_{0}, stable_{0}; 
  double target_x_{0},target_y_{0},target_z_{0},target_ego_x_{0},target_ego_y_{0},target_ego_z_{0};
  double held_altitude_enu_{0};
  bool held_altitude_valid_{false};
  bool target_override_valid_{false};
};
}  // namespace offboard_core_pkg
