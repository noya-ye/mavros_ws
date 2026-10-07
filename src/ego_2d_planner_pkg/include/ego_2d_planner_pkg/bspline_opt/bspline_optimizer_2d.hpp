#pragma once

#include <vector>
#include "ego_2d_planner_pkg/common/types.hpp"
#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"

namespace ego_2d_planner_pkg
{

class BsplineOptimizer2D
{
public:
  struct Result
  {
    bool success{false};
    double init_cost{0.0};
    double final_cost{0.0};
    int iterations{0};
  };

  void setParams(const PlannerParams2D& params);
  Result optimize(std::vector<Vec2>& ctrl_pts,
                  const std::vector<Vec2>& ref_ctrl_pts,
                  const GridMap2D& map) const;

private:
  double calcCostAndGradient(const std::vector<Vec2>& ctrl_pts,
                             const std::vector<Vec2>& ref_ctrl_pts,
                             const GridMap2D& map,
                             std::vector<Vec2>& grad) const;

  PlannerParams2D p_;
};

}  // namespace ego_2d_planner_pkg
