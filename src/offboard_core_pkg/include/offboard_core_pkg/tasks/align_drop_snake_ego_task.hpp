#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <vector>

#include "offboard_core_pkg/tasks/align_down_task.hpp"
#include "offboard_core_pkg/tasks/down_drop_task.hpp"
#include "offboard_core_pkg/tasks/ego_goto_task.hpp"
#include "offboard_core_pkg/tasks/land_task.hpp"
#include "offboard_core_pkg/tasks/red_cross_align_task.hpp"
#include "offboard_core_pkg/tasks/snake_grid_task.hpp"

namespace offboard_core_pkg {

class AlignDropSnakeEgoTask final : public ITask {
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
    double align_pixels_per_meter{100.0};
    int align_stable_frames{5};
    double align_arrive_distance_m{0.1};
    double align_max_step_m{0.10};
    double align_timeout_s{10.0};
    double contour_timeout_s{0.5};
    double align_loss_timeout_s{1.0};
    double align_loss_retry_cooldown_s{3.0};
    double align_trigger_cooldown_s{2.0};
    double align_retrigger_radius_m{0.55};
    bool red_cross_enabled{false};
    double red_cross_pixels_per_meter{100.0};
    int red_cross_stable_frames{5};
    double red_cross_arrive_distance_m{0.1};
    double red_cross_max_step_m{0.10};
    double red_cross_timeout_s{10.0};
    double drop_height_m{0.1};
    std::array<DownDropTask::obj_id, 3> drop_targets{{
      {0, 0.0, 0.10}, {1, -0.10, 0.0}, {2, 0.10, 0.0}}};
    std::string drop_serial_device{"/dev/ttyUSB0"};
    unsigned int drop_serial_baud_rate{115200};
  };

  AlignDropSnakeEgoTask(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
    const Config &cfg);

  std::string name() const override;
  void onEnter(Context &, MavrosIface &) override;
  Status tick(Context &, MavrosIface &, double dt_s) override;
  void onExit(Context &, MavrosIface &) override;

private:
  enum class Phase {
    SNAKE, AVOIDING, ALIGNING, ALIGNING_RED, DROPPING,
    RETURNING, LANDING, FINISHED, FAILED
  };
  enum class ResumePhase { SNAKE, AVOIDING };
  enum class AlignmentSource { DOWN, RED_CROSS };

  const char *phaseName() const;
  bool obstacleDataFresh(const Context &ctx) const;
  bool occupiedAt(const Context &ctx, double x, double y) const;
  bool obstacleNearSegment(const Context &ctx, double x0, double y0,
                           double x1, double y1) const;
  bool contourFresh(const Context &ctx) const;
  bool circleFresh(const Context &ctx) const;
  bool redCrossFresh(const Context &ctx) const;
  bool contourPositionEnu(const Context &ctx, Vec3 &position) const;
  bool circlePositionEnu(const Context &ctx, Vec3 &position) const;
  bool redCrossPositionEnu(const Context &ctx, Vec3 &position) const;
  bool insideCompletedDownTargetRadius(const Context &ctx) const;
  bool selectAvoidanceTarget(const Context &ctx);
  void beginAvoidance(Context &ctx, MavrosIface &iface);
  void beginAlignment(
    Context &ctx, MavrosIface &iface, ResumePhase resume_phase,
    AlignmentSource source);
  Status tickAlignment(Context &ctx, MavrosIface &iface, double dt_s);
  Status tickReturn(Context &ctx, MavrosIface &iface, double dt_s);
  Status tickLanding(Context &ctx, MavrosIface &iface, double dt_s);
  void failAndLand(Context &ctx, MavrosIface &iface, const char *reason);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  Config cfg_;
  SnakeGridTask snake_;
  EgoGotoTask ego_;
  AlignDownTask align_;
  RedCrossAlignTask red_cross_;
  DownDropTask down_drop_;
  LandTask land_;
  Phase phase_{Phase::FAILED};
  ResumePhase resume_phase_{ResumePhase::SNAKE};
  std::size_t avoidance_target_index_{0};
  double avoidance_elapsed_s_{0.0};
  double align_elapsed_s_{0.0};
  double align_loss_elapsed_s_{0.0};
  Vec3 alignment_entry_target_;
  bool alignment_entry_target_valid_{false};
  Vec3 lost_target_;
  double lost_target_cooldown_s_{0.0};
  double align_trigger_cooldown_s_{0.0};
  std::size_t drop_target_index_{0};
  bool finish_after_return_{false};
  std::string failure_reason_;
  Vec3 resume_position_;
  double resume_yaw_{0.0};
  bool alignment_latched_{false};
  bool red_cross_completed_{false};
  bool alignment_target_valid_{false};
  Vec3 alignment_target_enu_;
  std::vector<Vec3> completed_targets_enu_;
};

}  // namespace offboard_core_pkg
