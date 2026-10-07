#include "ego_2d_planner_pkg/diff_planner/polynomial_trajectory_2d.hpp"
#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_2d_planner_pkg
{
namespace
{
using Bezier = std::vector<Vec2>;

double binomial(int n, int k)
{
  double value = 1.0;
  for (int i = 1; i <= k; ++i) value *= static_cast<double>(n - i + 1) / i;
  return value;
}

Bezier toBezier(const PolynomialTrajectory2D::Piece& piece, int derivative)
{
  const int degree = 5 - derivative;
  Bezier points(degree + 1);
  for (int i = 0; i <= degree; ++i) {
    for (int k = 0; k <= i; ++k) {
      double factor = std::pow(piece.duration, k) * binomial(i, k) / binomial(degree, k);
      for (int d = 1; d <= derivative; ++d) factor *= k + d;
      points[i] += piece.coefficients[k + derivative] * factor;
    }
  }
  return points;
}

std::pair<Bezier, Bezier> split(const Bezier& points, double ratio = 0.5)
{
  Bezier work = points, left(points.size()), right(points.size());
  left[0] = work.front();
  right.back() = work.back();
  for (std::size_t level = 1; level < points.size(); ++level) {
    for (std::size_t i = 0; i < points.size() - level; ++i)
      work[i] = work[i] * (1.0 - ratio) + work[i + 1] * ratio;
    left[level] = work[0];
    right[points.size() - level - 1] = work[points.size() - level - 1];
  }
  return {left, right};
}

bool collisionFreeHull(const Bezier& points, const GridMap2D& map, int depth)
{
  Vec2 minimum = points.front(), maximum = points.front();
  for (const auto& point : points) {
    if (!finite(point)) return false;
    minimum.x = std::min(minimum.x, point.x); minimum.y = std::min(minimum.y, point.y);
    maximum.x = std::max(maximum.x, point.x); maximum.y = std::max(maximum.y, point.y);
  }
  // Include adjacent cells when the hull touches a grid boundary.
  const Vec2 epsilon{1e-10, 1e-10};
  int x0, y0, x1, y1;
  bool free = map.worldToGrid(minimum - epsilon, x0, y0) &&
              map.worldToGrid(maximum + epsilon, x1, y1);
  if (free) {
    for (int y = y0; y <= y1 && free; ++y)
      for (int x = x0; x <= x1; ++x)
        if (map.isOccupied(x, y)) { free = false; break; }
  }
  if (free) return true;
  if (depth == 18 || map.isOccupiedWorld(points.front()) || map.isOccupiedWorld(points.back()))
    return false;
  const auto halves = split(points);
  return collisionFreeHull(halves.first, map, depth + 1) &&
         collisionFreeHull(halves.second, map, depth + 1);
}

bool normBounded(const Bezier& points, double limit, int depth = 0)
{
  bool bounded = true;
  for (const auto& point : points) {
    if (!finite(point)) return false;
    if (norm(point) > limit * (1.0 + 1e-9)) bounded = false;
  }
  if (bounded) return true;
  if (depth == 16 || norm(points.front()) > limit * (1.0 + 1e-9) ||
      norm(points.back()) > limit * (1.0 + 1e-9)) return false;
  const auto halves = split(points);
  return normBounded(halves.first, limit, depth + 1) &&
         normBounded(halves.second, limit, depth + 1);
}
}  // namespace

bool PolynomialTrajectory2D::valid() const
{
  if (pieces.empty()) return false;
  double total = 0.0;
  for (const auto& piece : pieces) {
    if (!std::isfinite(piece.duration) || piece.duration <= 0.0) return false;
    total += piece.duration;
    for (const auto& coefficient : piece.coefficients) if (!finite(coefficient)) return false;
  }
  return std::isfinite(total);
}

double PolynomialTrajectory2D::duration() const
{
  double total = 0.0;
  for (const auto& piece : pieces) total += piece.duration;
  return total;
}

Vec2 PolynomialTrajectory2D::evaluate(double time, int derivative) const
{
  if (pieces.empty() || derivative < 0 || derivative > 5 || !std::isfinite(time)) return {};
  time = std::clamp(time, 0.0, duration());
  const Piece* selected = &pieces.back();
  for (const auto& piece : pieces) {
    selected = &piece;
    if (time <= piece.duration || &piece == &pieces.back()) break;
    time -= piece.duration;
  }
  time = std::min(time, selected->duration);
  Vec2 value;
  for (int k = 5; k >= derivative; --k) {
    double factor = 1.0;
    for (int d = 0; d < derivative; ++d) factor *= k - d;
    value = value * time + selected->coefficients[k] * factor;
  }
  return value;
}

std::vector<Vec2> PolynomialTrajectory2D::sample(double time_step, double from) const
{
  std::vector<Vec2> path;
  if (!valid()) return path;
  const double total = duration();
  from = std::clamp(from, 0.0, total);
  const int count = static_cast<int>(std::ceil((total - from) / std::max(0.01, time_step)));
  path.reserve(count + 1);
  for (int i = 0; i <= count; ++i)
    path.push_back(evaluate(count > 0 ? from + (total - from) * i / count : total));
  return path;
}

bool PolynomialTrajectory2D::collisionFree(const GridMap2D& map, double from) const
{
  if (!valid() || !std::isfinite(from)) return false;
  from = std::clamp(from, 0.0, duration());
  for (const auto& piece : pieces) {
    if (from >= piece.duration) { from -= piece.duration; continue; }
    auto points = toBezier(piece, 0);
    if (from > 0.0) points = split(points, from / piece.duration).second;
    if (!collisionFreeHull(points, map, 0)) return false;
    from = 0.0;
  }
  return !map.isOccupiedWorld(evaluate(duration()));
}

bool PolynomialTrajectory2D::feasible(double max_vel, double max_acc) const
{
  if (!valid() || !std::isfinite(max_vel) || !std::isfinite(max_acc) ||
      max_vel <= 0.0 || max_acc <= 0.0) return false;
  for (const auto& piece : pieces)
    if (!normBounded(toBezier(piece, 1), max_vel) ||
        !normBounded(toBezier(piece, 2), max_acc)) return false;
  return true;
}

Vec2 lookaheadPoint2D(const std::vector<Vec2>& path, const Vec2& position, double distance)
{
  if (path.empty()) return position;
  std::size_t closest = 0;
  for (std::size_t i = 1; i < path.size(); ++i)
    if (dist(path[i], position) < dist(path[closest], position)) closest = i;
  double remaining = std::max(0.0, distance);
  for (std::size_t i = closest + 1; i < path.size(); ++i) {
    const double length = dist(path[i - 1], path[i]);
    if (length >= remaining && length > 1e-12)
      return path[i - 1] + (path[i] - path[i - 1]) * (remaining / length);
    remaining -= length;
  }
  return path.back();
}
}  // namespace ego_2d_planner_pkg
