#include "ego_2d_planner_pkg/diff_planner/diff_optimizer_2d.hpp"
#include "ego_2d_planner_pkg/diff_planner/vendor/lbfgs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_2d_planner_pkg
{
namespace
{
// Diff-Planner's positive real time / unconstrained virtual time mapping.
double realTime(double virtual_time)
{
  return 0.05 + (virtual_time > 0.0 ? (0.5 * virtual_time + 1.0) * virtual_time + 1.0
                           : 1.0 / ((0.5 * virtual_time - 1.0) * virtual_time + 1.0));
}
double virtualTime(double real_time)
{
  const double positive_time = real_time - 0.05;
  return positive_time > 1.0 ? std::sqrt(2.0 * positive_time - 1.0) - 1.0
                        : 1.0 - std::sqrt(2.0 / positive_time - 1.0);
}
double timeDerivative(double virtual_time)
{
  const double denominator = (0.5 * virtual_time - 1.0) * virtual_time + 1.0;
  return virtual_time > 0.0 ? virtual_time + 1.0
                           : (1.0 - virtual_time) / (denominator * denominator);
}
Eigen::Vector3d spatial(const Vec2& point) { return {point.x, point.y, 0.0}; }
}  // namespace

void DiffOptimizer2D::generate(const double* x)
{
  for (int i = 0; i < pieces_ - 1; ++i) inner_.col(i) << x[2 * i], x[2 * i + 1], 0.0;
  for (int i = 0; i < pieces_; ++i) times_(i) = realTime(x[2 * (pieces_ - 1) + i]);
  minco_.generate(inner_, times_);
}

void DiffOptimizer2D::setAnchors(const GridMap2D& map)
{
  anchors_.assign(pieces_ * (samples_ + 1), {});
  const double radius = p_.dist0 + 2.0 * map.resolution();
  const int cells = static_cast<int>(std::ceil(radius / map.resolution()));
  for (int i = 0; i < pieces_; ++i) {
    for (int j = 0; j <= samples_; ++j) {
      const Vec2 reference = guide_[i] + (guide_[i + 1] - guide_[i]) * (double(j) / samples_);
      int gx = 0, gy = 0;
      if (!map.worldToGrid(reference, gx, gy)) continue;
      double nearest = radius;
      Vec2 obstacle;
      bool found = false;
      for (int y = std::max(0, gy - cells); y <= std::min(map.height() - 1, gy + cells); ++y) {
        for (int x = std::max(0, gx - cells); x <= std::min(map.width() - 1, gx + cells); ++x) {
          if (!map.isOccupied(x, y)) continue;
          const Vec2 center = map.gridToWorld(x, y);
          const double distance = dist(reference, center);
          if (distance < nearest) { nearest = distance; obstacle = center; found = true; }
        }
      }
      auto& constraints = anchors_[i * (samples_ + 1) + j];
      if (found && nearest > 1e-12) {
        const Vec2 direction = normalized(reference - obstacle);
        const double edge = 0.5 * map.resolution() / std::max(std::abs(direction.x), std::abs(direction.y));
        constraints.push_back({obstacle + direction * edge, direction});
      }
      // Unknown space beyond the rolling map is a hard boundary in final acceptance.
      const double x0 = map.origin_x(), y0 = map.origin_y();
      const double x1 = x0 + map.width() * map.resolution(), y1 = y0 + map.height() * map.resolution();
      if (reference.x - x0 < radius) constraints.push_back({{x0, reference.y}, {1, 0}});
      if (x1 - reference.x < radius) constraints.push_back({{x1, reference.y}, {-1, 0}});
      if (reference.y - y0 < radius) constraints.push_back({{reference.x, y0}, {0, 1}});
      if (y1 - reference.y < radius) constraints.push_back({{reference.x, y1}, {0, -1}});
    }
  }
}

double DiffOptimizer2D::objective(const double* x, double* gradient)
{
  for (int i = 0; i < 3 * pieces_ - 2; ++i) {
    if (!std::isfinite(x[i])) {
      std::fill(gradient, gradient + 3 * pieces_ - 2, 0.0);
      return std::numeric_limits<double>::infinity();
    }
  }
  for (int i = 0; i < pieces_; ++i) {
    if (realTime(x[2 * (pieces_ - 1) + i]) > 120.0) {
      std::fill(gradient, gradient + 3 * pieces_ - 2, 0.0);
      return std::numeric_limits<double>::infinity();
    }
  }
  generate(x);
  double cost = 0.0;
  minco_.initGradCost(time_gradient_, cost);
  cost *= p_.lambda_smooth;
  minco_.get_gdC() *= p_.lambda_smooth;
  time_gradient_ *= p_.lambda_smooth;
  const auto& coefficients = minco_.get_b();
  for (int i = 0; i < pieces_; ++i) {
    const auto c = coefficients.block<6, 3>(6 * i, 0);
    const double step = times_(i) / samples_;
    for (int j = 0; j <= samples_; ++j) {
      const double alpha = double(j) / samples_;
      const double t = alpha * times_(i), t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
      Eigen::Matrix<double, 6, 1> b0, b1, b2, b3;
      b0 << 1, t, t2, t3, t4, t5;
      b1 << 0, 1, 2*t, 3*t2, 4*t3, 5*t4;
      b2 << 0, 0, 2, 6*t, 12*t2, 20*t3;
      b3 << 0, 0, 0, 6, 24*t, 60*t2;
      const Eigen::Vector3d pos = c.transpose() * b0, vel = c.transpose() * b1;
      const Eigen::Vector3d acc = c.transpose() * b2, jerk = c.transpose() * b3;
      const Vec2 position{pos.x(), pos.y()};
      const Vec2 reference = guide_[i] + (guide_[i + 1] - guide_[i]) * alpha;
      Eigen::Vector3d gp = 2.0 * p_.lambda_fitness * (pos - spatial(reference));
      Eigen::Vector3d gv = Eigen::Vector3d::Zero(), ga = Eigen::Vector3d::Zero();
      double penalty = p_.lambda_fitness * norm2(position - reference);
      for (const auto& anchor : anchors_[i * (samples_ + 1) + j]) {
        const double violation = p_.dist0 - dot(position - anchor.base, anchor.direction);
        if (violation <= 0.0) continue;
        penalty += p_.diff_weight_collision * violation * violation * violation;
        gp -= spatial(anchor.direction) * (3.0 * p_.diff_weight_collision * violation * violation);
      }
      const double velocity_violation = vel.squaredNorm() / (p_.max_vel * p_.max_vel) - 1.0;
      const double acceleration_violation = acc.squaredNorm() / (p_.max_acc * p_.max_acc) - 1.0;
      if (velocity_violation > 0.0) {
        penalty += p_.diff_weight_feasibility * std::pow(velocity_violation, 3);
        gv = vel * (6.0 * p_.diff_weight_feasibility * velocity_violation * velocity_violation /
                    (p_.max_vel * p_.max_vel));
      }
      if (acceleration_violation > 0.0) {
        penalty += p_.diff_weight_feasibility * std::pow(acceleration_violation, 3);
        ga = acc * (6.0 * p_.diff_weight_feasibility * acceleration_violation * acceleration_violation /
                    (p_.max_acc * p_.max_acc));
      }
      const double weight = (j == 0 || j == samples_) ? 0.5 : 1.0;
      cost += weight * step * penalty;
      minco_.get_gdC().block<6, 3>(6 * i, 0) +=
        weight * step * (b0 * gp.transpose() + b1 * gv.transpose() + b2 * ga.transpose());
      time_gradient_(i) += weight * (penalty / samples_ +
        step * alpha * (gp.dot(vel) + gv.dot(acc) + ga.dot(jerk)));
    }
  }
  point_gradient_.setZero();
  minco_.getGrad2TP(time_gradient_, point_gradient_);
  for (int i = 0; i < pieces_ - 1; ++i) {
    gradient[2*i] = point_gradient_(0, i);
    gradient[2*i+1] = point_gradient_(1, i);
  }
  for (int i = 0; i < pieces_; ++i) {
    const int index = 2 * (pieces_ - 1) + i;
    gradient[index] = (time_gradient_(i) + p_.diff_weight_time) * timeDerivative(x[index]);
  }
  return cost + p_.diff_weight_time * times_.sum();
}

double DiffOptimizer2D::costCallback(void* instance, const double* x, double* gradient, int n)
{
  auto& optimizer = *static_cast<DiffOptimizer2D*>(instance);
  if (optimizer.evaluations_ >= optimizer.p_.diff_max_evaluations ||
      std::chrono::steady_clock::now() >= optimizer.deadline_) {
    std::fill(gradient, gradient + n, 0.0);
    return std::numeric_limits<double>::infinity();
  }
  ++optimizer.evaluations_;
  const double cost = optimizer.objective(x, gradient);
  bool finite_gradient = true;
  for (int i = 0; i < n; ++i) if (!std::isfinite(gradient[i])) finite_gradient = false;
  if (std::isfinite(cost) && finite_gradient && cost < optimizer.best_cost_) {
    optimizer.best_cost_ = cost;
    optimizer.best_x_.assign(x, x + n);
  }
  if (!std::isfinite(cost) || !finite_gradient) {
    std::fill(gradient, gradient + n, 0.0);
    return std::numeric_limits<double>::infinity();
  }
  return cost;
}

int DiffOptimizer2D::progressCallback(void* instance, const double*, const double*, double, double,
                                    double, double, int, int iteration, int)
{
  auto& optimizer = *static_cast<DiffOptimizer2D*>(instance);
  optimizer.iterations_ = iteration;
  return optimizer.evaluations_ >= optimizer.p_.diff_max_evaluations ||
         std::chrono::steady_clock::now() >= optimizer.deadline_;
}

PolynomialTrajectory2D DiffOptimizer2D::trajectory() const
{
  PolynomialTrajectory2D output;
  const auto& c = minco_.get_b();
  output.pieces.resize(pieces_);
  for (int i = 0; i < pieces_; ++i) {
    output.pieces[i].duration = times_(i);
    for (int k = 0; k < 6; ++k) output.pieces[i].coefficients[k] = {c(6*i+k, 0), c(6*i+k, 1)};
  }
  return output;
}

DiffOptimizer2D::Result DiffOptimizer2D::optimize(
  const GridMap2D& map, const std::vector<Vec2>& guide, const Vec2& start_velocity,
  const Vec2& start_acceleration, const PlannerParams2D& params,
  std::chrono::steady_clock::time_point deadline)
{
  Result result;
  p_ = params; guide_ = guide; deadline_ = deadline;
  // Upstream's single-piece shortcut does not initialize the adjoint system.
  // Two pieces also keep short/zero-distance goals on the regular MINCO path.
  if (guide_.size() == 2) guide_.insert(guide_.begin() + 1, (guide_.front() + guide_.back()) * 0.5);
  pieces_ = static_cast<int>(guide_.size()) - 1;
  if (pieces_ < 1 || pieces_ > 64 || !finite(start_velocity) || !finite(start_acceleration)) return result;
  samples_ = std::max(4, p_.diff_samples_per_piece);
  evaluations_ = iterations_ = 0;
  times_.resize(pieces_); time_gradient_.resize(pieces_);
  inner_.resize(3, pieces_ - 1); point_gradient_.resize(3, pieces_ - 1);
  Eigen::Matrix3d head = Eigen::Matrix3d::Zero(), tail = Eigen::Matrix3d::Zero();
  head.col(0) = spatial(guide_.front()); head.col(1) = spatial(start_velocity); head.col(2) = spatial(start_acceleration);
  tail.col(0) = spatial(guide_.back());
  minco_.reset(head, tail, pieces_);
  const int variables = 2 * (pieces_ - 1) + pieces_;
  std::vector<double> x(variables), gradient(variables);
  for (int i = 0; i < pieces_ - 1; ++i) { x[2*i] = guide_[i+1].x; x[2*i+1] = guide_[i+1].y; }
  for (int i = 0; i < pieces_; ++i) {
    const double length = dist(guide_[i], guide_[i + 1]);
    const double duration = std::max({0.30, 1.9 * length / p_.max_vel, std::sqrt(6.0 * length / p_.max_acc)});
    x[2 * (pieces_ - 1) + i] = virtualTime(duration);
  }
  setAnchors(map);
  best_cost_ = std::numeric_limits<double>::infinity(); best_x_.clear();
  result.initial_cost = costCallback(this, x.data(), gradient.data(), variables);
  lbfgs::lbfgs_parameter_t settings;
  lbfgs::lbfgs_load_default_parameters(&settings);
  settings.mem_size = 8; settings.max_iterations = std::max(1, p_.optimizer_max_iter);
  settings.max_linesearch = 20; settings.past = 3; settings.delta = 1e-5;
  double cost = result.initial_cost;
  result.solver_status = lbfgs::lbfgs_optimize(variables, x.data(), &cost, costCallback,
                                             nullptr, progressCallback, this, &settings);
  result.evaluations = evaluations_; result.iterations = iterations_; result.final_cost = best_cost_;
  if (best_x_.empty()) return result;
  generate(best_x_.data());
  // Timing repair regenerates MINCO with unchanged boundary P/V/A. It never scales
  // an executable polynomial in place (that would change a moving start state).
  for (int attempt = 0; attempt < 8; ++attempt) {
    result.trajectory = trajectory();
    if (result.trajectory.feasible(p_.max_vel, p_.max_acc)) break;
    times_ *= 1.15;
    minco_.generate(inner_, times_);
  }
  return result;
}
}  // namespace ego_2d_planner_pkg
