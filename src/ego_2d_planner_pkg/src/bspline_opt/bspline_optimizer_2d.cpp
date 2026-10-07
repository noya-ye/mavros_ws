#include "ego_2d_planner_pkg/bspline_opt/bspline_optimizer_2d.hpp"

namespace ego_2d_planner_pkg
{

void BsplineOptimizer2D::setParams(const PlannerParams2D& params)
{
  p_ = params;
}

BsplineOptimizer2D::Result BsplineOptimizer2D::optimize(std::vector<Vec2>& ctrl_pts,
                                                        const std::vector<Vec2>& ref_ctrl_pts,
                                                        const GridMap2D& map) const
{
  Result res;
  if (ctrl_pts.size() < 7 || ctrl_pts.size() != ref_ctrl_pts.size()) {
    return res;
  }

  std::vector<Vec2> grad(ctrl_pts.size());
  res.init_cost = calcCostAndGradient(ctrl_pts, ref_ctrl_pts, map, grad);
  res.final_cost = res.init_cost;

  const int anchor = 3;  // clamped endpoint duplicates: do not move them
  for (int iter = 0; iter < p_.optimizer_max_iter; ++iter) {
    const double cost = iter == 0 ? res.init_cost :
      calcCostAndGradient(ctrl_pts, ref_ctrl_pts, map, grad);
    res.final_cost = cost;
    res.iterations = iter + 1;

    double max_g = 0.0;
    for (std::size_t i = anchor; i + anchor < ctrl_pts.size(); ++i) {
      max_g = std::max(max_g, norm(grad[i]));
    }
    if (max_g < 1e-5) {
      break;
    }

    for (std::size_t i = anchor; i + anchor < ctrl_pts.size(); ++i) {
      Vec2 update = grad[i] * (-p_.optimizer_step_size);
      update = clampNorm(update, p_.optimizer_max_update);
      ctrl_pts[i] += update;
    }
  }

  res.final_cost = calcCostAndGradient(ctrl_pts, ref_ctrl_pts, map, grad);
  res.success = std::isfinite(res.final_cost);
  return res;
}

double BsplineOptimizer2D::calcCostAndGradient(const std::vector<Vec2>& q,
                                              const std::vector<Vec2>& ref,
                                              const GridMap2D& map,
                                              std::vector<Vec2>& grad) const
{
  const std::size_t n = q.size();
  grad.assign(n, Vec2{});
  double cost = 0.0;

  // 1) Smoothness cost: sum ||q[i-1] - 2q[i] + q[i+1]||^2
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const Vec2 d = q[i - 1] - q[i] * 2.0 + q[i + 1];
    const double c = norm2(d);
    cost += p_.lambda_smooth * c;
    const Vec2 gd = d * (2.0 * p_.lambda_smooth);
    grad[i - 1] += gd;
    grad[i]     -= gd * 2.0;
    grad[i + 1] += gd;
  }

  // 2) Collision cost: push control points out of distance threshold dist0.
  //    Distance field is already computed on inflated occupancy grid.
  for (std::size_t i = 3; i + 3 < n; ++i) {
    const double d = map.getDistanceWorld(q[i]);
    if (d < p_.dist0) {
      const double e = p_.dist0 - d;
      cost += p_.lambda_collision * e * e;
      const Vec2 gdist = map.getDistanceGradientWorld(q[i]);
      grad[i] += gdist * (-2.0 * p_.lambda_collision * e);
    }
  }

  // 3) Feasibility cost: approximate velocity and acceleration constraints.
  const double max_step = std::max(0.05, p_.max_vel * p_.knot_span);
  const double max_acc_step = std::max(0.05, p_.max_acc * p_.knot_span * p_.knot_span);

  for (std::size_t i = 0; i + 1 < n; ++i) {
    const Vec2 v = q[i + 1] - q[i];
    const double l = norm(v);
    if (l > max_step) {
      const double e = l - max_step;
      const Vec2 u = normalized(v);
      cost += p_.lambda_feasibility * e * e;
      grad[i]     -= u * (2.0 * p_.lambda_feasibility * e);
      grad[i + 1] += u * (2.0 * p_.lambda_feasibility * e);
    }
  }

  for (std::size_t i = 1; i + 1 < n; ++i) {
    const Vec2 a = q[i - 1] - q[i] * 2.0 + q[i + 1];
    const double l = norm(a);
    if (l > max_acc_step) {
      const double e = l - max_acc_step;
      const Vec2 u = normalized(a);
      cost += p_.lambda_feasibility * e * e;
      const Vec2 ga = u * (2.0 * p_.lambda_feasibility * e);
      grad[i - 1] += ga;
      grad[i]     -= ga * 2.0;
      grad[i + 1] += ga;
    }
  }

  // 4) Fitness cost: do not deviate too far from A* initialized control points.
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 d = q[i] - ref[i];
    cost += p_.lambda_fitness * norm2(d);
    grad[i] += d * (2.0 * p_.lambda_fitness);
  }

  return cost;
}

}  // namespace ego_2d_planner_pkg
