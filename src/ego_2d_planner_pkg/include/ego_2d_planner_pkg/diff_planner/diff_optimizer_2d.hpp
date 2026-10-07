#pragma once

#include <chrono>
#include <Eigen/Core>
#include "ego_2d_planner_pkg/diff_planner/polynomial_trajectory_2d.hpp"
#include "ego_2d_planner_pkg/diff_planner/vendor/poly_traj_utils.hpp"
#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"

namespace ego_2d_planner_pkg
{
class DiffOptimizer2D
{
public:
  struct Result
  {
    PolynomialTrajectory2D trajectory;
    double initial_cost{0.0}, final_cost{0.0};
    int evaluations{0}, iterations{0}, solver_status{0};
  };

  Result optimize(const GridMap2D& map, const std::vector<Vec2>& guide,
                  const Vec2& start_velocity, const Vec2& start_acceleration,
                  const PlannerParams2D& params,
                  std::chrono::steady_clock::time_point deadline);

private:
  friend struct DiffPlannerTestAccess;
  struct Anchor { Vec2 base, direction; };
  static double costCallback(void* instance, const double* x, double* gradient, int n);
  static int progressCallback(void* instance, const double*, const double*, double, double,
                              double, double, int, int iteration, int);
  double objective(const double* x, double* gradient);
  void setAnchors(const GridMap2D& map);
  void generate(const double* x);
  PolynomialTrajectory2D trajectory() const;

  PlannerParams2D p_;
  int pieces_{0}, samples_{0}, evaluations_{0}, iterations_{0};
  poly_traj::MinJerkOpt minco_;
  std::vector<Vec2> guide_;
  std::vector<std::vector<Anchor>> anchors_;
  Eigen::VectorXd times_, time_gradient_;
  Eigen::MatrixXd inner_, point_gradient_;
  std::vector<double> best_x_;
  double best_cost_{0.0};
  std::chrono::steady_clock::time_point deadline_;
};
}  // namespace ego_2d_planner_pkg
