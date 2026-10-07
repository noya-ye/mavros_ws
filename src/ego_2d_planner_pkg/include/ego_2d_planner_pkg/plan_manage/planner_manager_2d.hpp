#pragma once

#include <vector>
#include <string>
#include "ego_2d_planner_pkg/common/types.hpp"
#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"
#include "ego_2d_planner_pkg/path_searching/astar_2d.hpp"
#include "ego_2d_planner_pkg/diff_planner/polynomial_trajectory_2d.hpp"

namespace ego_2d_planner_pkg
{

class PlannerManager2D
{
public:
  enum class PlanStatus
  {
    FAILED = 0,
    DIFF_SUCCESS,
    ASTAR_FAILED,
    PATH_TOO_SHORT,
    INVALID_INPUT,
    START_OR_GOAL_OCCUPIED,
    OPTIMIZATION_FAILED,
    COLLISION_AFTER_OPTIMIZATION,
    DYNAMICALLY_INFEASIBLE
  };

  struct PlanResult
  {
    bool success{false};
    bool used_optimizer{false};
    bool smooth_safe{false};
    bool raw_path_safe{false};
    bool fallback_raw{false};  // kept only for diagnostics; normal EGO-style logic does not use it as success
    PlanStatus status{PlanStatus::FAILED};
    std::string message;

    Vec2 local_goal;
    std::vector<Vec2> raw_path;
    std::vector<Vec2> smooth_path;
    std::vector<Vec2> selected_path;

    PolynomialTrajectory2D trajectory;

    double init_cost{0.0};
    double final_cost{0.0};
    int optimizer_iter{0};
    int rebound_attempt{0};
    int optimizer_evaluations{0};
    double planning_ms{0.0};
  };

  void setParams(const PlannerParams2D& params);

  // ROS adapter retains historical entry points; both use Diff MINCO.
  PlanResult planGlobalTraj(const GridMap2D& map, const Vec2& start, const Vec2& goal);

  // Raw A* is only a guide. Every executable quintic passes hard acceptance.
  PlanResult reboundReplan(const GridMap2D& map, const Vec2& start, const Vec2& goal, bool init,
                           const Vec2& start_vel = {}, const Vec2& start_acc = {});

  // Backward-compatible alias.
  PlanResult plan(const GridMap2D& map, const Vec2& start, const Vec2& goal)
  {
    return reboundReplan(map, start, goal, true);
  }

private:
  bool checkInput(const GridMap2D& map, const Vec2& start, const Vec2& goal, PlanResult& out) const;

  PlannerParams2D p_;
  AStar2D astar_;
};

}  // namespace ego_2d_planner_pkg
