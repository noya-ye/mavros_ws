#include "ego_2d_planner_pkg/plan_manage/planner_manager_2d.hpp"
#include "ego_2d_planner_pkg/diff_planner/diff_optimizer_2d.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace ego_2d_planner_pkg
{
namespace
{
bool segmentFree(const GridMap2D& map, const Vec2& a, const Vec2& b)
{
  PolynomialTrajectory2D line;
  PolynomialTrajectory2D::Piece piece;
  piece.duration = 1.0; piece.coefficients[0] = a; piece.coefficients[1] = b - a;
  line.pieces.push_back(piece);
  return line.collisionFree(map);
}

std::vector<Vec2> makeGuide(const std::vector<Vec2>& path, double spacing)
{
  std::vector<Vec2> corners{path.front()};
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const Vec2 before = normalized(path[i] - path[i-1]), after = normalized(path[i+1] - path[i]);
    if (norm(before - after) > 1e-6) corners.push_back(path[i]);
  }
  corners.push_back(path.back());
  std::vector<Vec2> guide{corners.front()};
  for (std::size_t i = 1; i < corners.size(); ++i) {
    const int count = std::max(1, static_cast<int>(std::ceil(dist(corners[i-1], corners[i]) / spacing)));
    for (int j = 1; j <= count; ++j)
      guide.push_back(corners[i-1] + (corners[i] - corners[i-1]) * (double(j) / count));
  }
  return guide;
}
}  // namespace

void PlannerManager2D::setParams(const PlannerParams2D& params)
{
  p_ = params;
  if (!std::isfinite(p_.max_vel) || p_.max_vel <= 0.0 ||
      !std::isfinite(p_.max_acc) || p_.max_acc <= 0.0 ||
      !std::isfinite(p_.diff_piece_length) || p_.diff_piece_length <= 0.0 ||
      !std::isfinite(p_.diff_max_solve_ms) || p_.diff_max_solve_ms <= 0.0 ||
      p_.diff_samples_per_piece < 4 || p_.diff_samples_per_piece > 64 ||
      p_.diff_max_evaluations < 1 || p_.max_rebound_attempts < 1 ||
      !std::isfinite(p_.dist0) || p_.dist0 < 0.0) {
    throw std::invalid_argument("invalid Diff-Planner timing, sampling or dynamic limits");
  }
  for (double weight : {p_.lambda_smooth, p_.lambda_fitness, p_.diff_weight_time,
                        p_.diff_weight_collision, p_.diff_weight_feasibility})
    if (!std::isfinite(weight) || weight < 0.0) throw std::invalid_argument("invalid Diff-Planner weight");
}

bool PlannerManager2D::checkInput(const GridMap2D& map, const Vec2& start,
                                 const Vec2& goal, PlanResult& out) const
{
  out.local_goal = start;
  int sx, sy, gx, gy;
  if (!finite(start) || !finite(goal) || !map.worldToGrid(start, sx, sy) || !map.worldToGrid(goal, gx, gy)) {
    out.status = PlanStatus::INVALID_INPUT; out.message = "invalid or out-of-map start/goal"; return false;
  }
  if (map.isOccupied(sx, sy) || map.isOccupied(gx, gy)) {
    out.status = PlanStatus::START_OR_GOAL_OCCUPIED;
    out.message = "start or requested goal occupied; target retained"; return false;
  }
  return true;
}

PlannerManager2D::PlanResult PlannerManager2D::planGlobalTraj(
  const GridMap2D& map, const Vec2& start, const Vec2& goal)
{
  return reboundReplan(map, start, goal, true);
}

PlannerManager2D::PlanResult PlannerManager2D::reboundReplan(
  const GridMap2D& map, const Vec2& start, const Vec2& goal, bool init,
  const Vec2& start_vel, const Vec2& start_acc)
{
  PlanResult out;
  const auto begin = std::chrono::steady_clock::now();
  const auto deadline = begin + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(p_.diff_max_solve_ms));
  if (!checkInput(map, start, goal, out)) return out;
  if (!finite(start_vel) || !finite(start_acc) || norm(start_vel) > p_.max_vel * (1.0 + 1e-9) ||
      norm(start_acc) > p_.max_acc * (1.0 + 1e-9)) {
    out.status = PlanStatus::DYNAMICALLY_INFEASIBLE;
    out.message = "initial velocity or acceleration already exceeds configured limit"; return out;
  }
  if (!p_.optimizer_enable) {
    out.status = PlanStatus::OPTIMIZATION_FAILED; out.message = "Diff optimizer disabled"; return out;
  }
  if (dist(start, goal) < map.resolution() && segmentFree(map, start, goal)) {
    out.raw_path = {start, goal};
  } else if (!astar_.search(map, start, goal, out.raw_path, p_.astar_max_iter)) {
    out.status = PlanStatus::ASTAR_FAILED; out.message = "2D guide search failed"; return out;
  }
  if (out.raw_path.size() < 2) {
    out.status = PlanStatus::PATH_TOO_SHORT; out.message = "guide has no executable segment"; return out;
  }
  out.raw_path_safe = true;
  for (std::size_t i = 1; i < out.raw_path.size(); ++i) {
    if (!segmentFree(map, out.raw_path[i-1], out.raw_path[i])) { out.raw_path_safe = false; break; }
  }
  if (!out.raw_path_safe) {
    out.status = PlanStatus::ASTAR_FAILED; out.message = "guide intersects occupied grid cells"; return out;
  }
  const auto guide = makeGuide(out.raw_path, p_.diff_piece_length);
  int remaining_evaluations = p_.diff_max_evaluations;
  for (int k = 0; k < p_.max_rebound_attempts && remaining_evaluations > 0; ++k) {
    if (std::chrono::steady_clock::now() >= deadline) break;
    PlannerParams2D trial = p_;
    trial.diff_max_evaluations = remaining_evaluations;
    trial.diff_weight_collision *= std::pow(p_.retry_collision_scale, k);
    trial.lambda_fitness *= std::pow(2.0, k);
    trial.lambda_smooth *= std::pow(p_.retry_smooth_scale, k);
    DiffOptimizer2D optimizer;
    auto optimized = optimizer.optimize(map, guide, start_vel, start_acc, trial, deadline);
    out.optimizer_iter += optimized.iterations;
    out.optimizer_evaluations += optimized.evaluations;
    remaining_evaluations -= optimized.evaluations;
    out.rebound_attempt = k + 1;
    out.init_cost = optimized.initial_cost; out.final_cost = optimized.final_cost;
    if (!optimized.trajectory.valid()) continue;
    out.smooth_path = optimized.trajectory.sample(p_.bspline_sample_step);
    if (!optimized.trajectory.feasible(p_.max_vel, p_.max_acc)) {
      out.status = PlanStatus::DYNAMICALLY_INFEASIBLE; continue;
    }
    if (!optimized.trajectory.collisionFree(map)) {
      out.status = PlanStatus::COLLISION_AFTER_OPTIMIZATION; continue;
    }
    out.trajectory = std::move(optimized.trajectory);
    out.selected_path = out.smooth_path;
    out.success = out.used_optimizer = out.smooth_safe = true;
    out.status = PlanStatus::DIFF_SUCCESS;
    out.local_goal = lookaheadPoint2D(out.selected_path, start, p_.lookahead_dist);
    out.message = init ? "Diff MINCO initial trajectory accepted" : "Diff MINCO replan accepted";
    break;
  }
  out.planning_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  if (!out.success) out.message = "Diff solve failed hard acceptance or exhausted budget; no raw fallback";
  return out;
}
}  // namespace ego_2d_planner_pkg