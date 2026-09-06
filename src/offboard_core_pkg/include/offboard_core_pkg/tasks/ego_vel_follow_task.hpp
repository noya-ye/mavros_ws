#pragma once
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/planners/ego_vel_planner.hpp"
#include <rclcpp/rclcpp.hpp>
namespace offboard_core_pkg {
class EgoVelFollowTask final : public ITask {
public:
  using Config = EgoVelPlanner::Config;
  explicit EgoVelFollowTask(rclcpp::Logger logger, const Config &cfg = Config{}) : logger_(logger), planner_(cfg) {}
  std::string name() const override { return "EGO_VEL_FOLLOW"; }
  void onEnter(Context &ctx, MavrosIface &) override { planner_.reset(ctx); }
  Status tick(Context &ctx, MavrosIface &, double) override { EgoVelPlanner::Debug d; planner_.plan(ctx, &d); return Status::RUNNING; }
private: rclcpp::Logger logger_; EgoVelPlanner planner_;
};
}  // namespace offboard_core_pkg
