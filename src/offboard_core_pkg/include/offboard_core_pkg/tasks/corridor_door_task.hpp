#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/tasks/ego_goto_task.hpp"
#include "offboard_core_pkg/tasks/goto_task.hpp"

namespace offboard_core_pkg {

class CorridorDoorTask final : public ITask {
public:
  struct Config {
    int door_count{1};
    int occupied_threshold{50};
    double min_opening_width_m{0.5};
    double max_opening_width_m{1.5};
    double min_lookahead_m{0.5};
    double max_lookahead_m{5.0};
    double occupancy_timeout_s{0.5};
    double goto_tolerance_m{0.08};
    double crossing_distance_m{0.1};
    double stage_timeout_s{30.0};
    EgoGotoTask::Config ego;
  };

  CorridorDoorTask(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
                   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
                   const Config &cfg);
  std::string name() const override { return "CORRIDOR_DOOR"; }
  void onEnter(Context &ctx, MavrosIface &iface) override;
  Status tick(Context &ctx, MavrosIface &iface, double dt_s) override;
  void onExit(Context &ctx, MavrosIface &iface) override;
  bool canPause(const Context &) const override { return false; }

  int doorsCrossed() const { return doors_crossed_; }

private:
  enum class Phase { FINDING, APPROACHING, CROSSING, FAILED };
  bool mapFresh(const Context &ctx) const;
  bool findOpening(const Context &ctx, double &x, double &y) const;
  void hold(Context &ctx) const;
  void fail(Context &ctx, const char *reason);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  Config cfg_;
  EgoGotoTask ego_;
  GotoTask goto_{0.0, 0.0, 0.0, 0.08};
  Phase phase_{Phase::FAILED};
  int doors_crossed_{0};
  double phase_elapsed_s_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
};

}  // namespace offboard_core_pkg
