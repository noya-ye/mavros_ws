#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/tasks/ego_goto_task.hpp"
#include "offboard_core_pkg/tasks/land_task.hpp"
#include "offboard_core_pkg/tasks/snake_grid_task.hpp"

namespace offboard_core_pkg {

class SnakeEgoAvoidTask final : public ITask {
public:
  struct Config {
    SnakeGridTask::Config snake;
    EgoGotoTask::Config ego;
    double trigger_distance_m{0.8};
    double occupancy_timeout_s{0.5};
    double avoidance_timeout_s{30.0};
    int occupied_threshold{50};
    double landing_timeout_s{15.0};
    double landing_retry_interval_s{1.0};
  };

  SnakeEgoAvoidTask(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
    const Config &cfg);

  std::string name() const override;
  void onEnter(Context &, MavrosIface &) override;
  Status tick(Context &, MavrosIface &, double dt_s) override;
  void onExit(Context &, MavrosIface &) override;

private:
  enum class Phase { SNAKE, AVOIDING, LANDING, FAILED };

  bool obstacleDataFresh(const Context &ctx) const;
  bool occupiedAt(const Context &ctx, double x, double y) const;
  bool obstacleNearSegment(const Context &ctx, double x0, double y0,
                           double x1, double y1) const;
  bool selectAvoidanceTarget(const Context &ctx);
  void beginAvoidance(Context &ctx, MavrosIface &iface);
  Status tickLanding(Context &ctx, MavrosIface &iface, double dt_s);
  void failAndLand(Context &ctx, MavrosIface &iface, const char *reason);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  Config cfg_;
  SnakeGridTask snake_;
  EgoGotoTask ego_;
  LandTask land_;
  Phase phase_{Phase::FAILED};
  std::size_t avoidance_target_index_{0};
  double avoidance_elapsed_s_{0.0};
  std::string failure_reason_;
};

}  // namespace offboard_core_pkg
