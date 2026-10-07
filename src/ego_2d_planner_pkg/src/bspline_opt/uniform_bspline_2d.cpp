#include "ego_2d_planner_pkg/bspline_opt/uniform_bspline_2d.hpp"

namespace ego_2d_planner_pkg
{

UniformBspline2D::UniformBspline2D(const std::vector<Vec2>& ctrl_pts, double knot_span)
{
  setControlPoints(ctrl_pts, knot_span);
}

void UniformBspline2D::setControlPoints(const std::vector<Vec2>& ctrl_pts, double knot_span)
{
  ctrl_pts_ = ctrl_pts;
  knot_span_ = std::max(0.05, knot_span);
}

double UniformBspline2D::duration() const
{
  if (ctrl_pts_.size() < 4) return 0.0;
  return static_cast<double>(ctrl_pts_.size() - 3) * knot_span_;
}

Vec2 UniformBspline2D::evaluate(double t) const
{
  if (ctrl_pts_.empty()) return {};
  if (ctrl_pts_.size() < 4) return ctrl_pts_.front();

  t = std::clamp(t, 0.0, duration());
  int seg = static_cast<int>(std::floor(t / knot_span_));
  if (seg > static_cast<int>(ctrl_pts_.size()) - 4) seg = static_cast<int>(ctrl_pts_.size()) - 4;
  const double u = std::clamp((t - static_cast<double>(seg) * knot_span_) / knot_span_, 0.0, 1.0);

  const Vec2& p0 = ctrl_pts_[seg];
  const Vec2& p1 = ctrl_pts_[seg + 1];
  const Vec2& p2 = ctrl_pts_[seg + 2];
  const Vec2& p3 = ctrl_pts_[seg + 3];

  const double u2 = u * u;
  const double u3 = u2 * u;
  const double b0 = (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0;
  const double b1 = (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0;
  const double b2 = (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0;
  const double b3 = u3 / 6.0;

  return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

std::vector<Vec2> UniformBspline2D::sample(double step) const
{
  std::vector<Vec2> out;
  const double dur = duration();
  if (dur <= 0.0) return ctrl_pts_;
  step = std::max(0.02, step);
  const int n = std::max(2, static_cast<int>(std::ceil(dur / step)));
  out.reserve(n + 1);
  for (int i = 0; i <= n; ++i) {
    const double t = dur * static_cast<double>(i) / static_cast<double>(n);
    out.push_back(evaluate(t));
  }
  return out;
}

std::vector<Vec2> UniformBspline2D::resamplePolyline(const std::vector<Vec2>& path, double step)
{
  if (path.size() <= 1) return path;
  step = std::max(0.02, step);

  std::vector<double> acc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    acc[i] = acc[i - 1] + dist(path[i - 1], path[i]);
  }
  const double total = acc.back();
  if (total < 1e-6) return {path.front(), path.back()};

  const int n = std::max(2, static_cast<int>(std::ceil(total / step)) + 1);
  std::vector<Vec2> out;
  out.reserve(n);

  std::size_t seg = 1;
  for (int k = 0; k < n; ++k) {
    const double s = total * static_cast<double>(k) / static_cast<double>(n - 1);
    while (seg < acc.size() - 1 && acc[seg] < s) ++seg;
    const double denom = std::max(1e-9, acc[seg] - acc[seg - 1]);
    const double u = std::clamp((s - acc[seg - 1]) / denom, 0.0, 1.0);
    out.push_back(path[seg - 1] + (path[seg] - path[seg - 1]) * u);
  }
  out.front() = path.front();
  out.back() = path.back();
  return out;
}

std::vector<Vec2> UniformBspline2D::makeClampedControlPoints(const std::vector<Vec2>& path, double ctrl_dist)
{
  if (path.empty()) return {};
  std::vector<Vec2> key = resamplePolyline(path, ctrl_dist);
  if (key.size() < 2) key.push_back(path.back());

  std::vector<Vec2> ctrl;
  ctrl.reserve(key.size() + 4);

  // cubic B-spline clamped endpoints: S,S,S, ..., G,G,G
  ctrl.push_back(key.front());
  ctrl.push_back(key.front());
  ctrl.push_back(key.front());

  for (std::size_t i = 1; i + 1 < key.size(); ++i) {
    ctrl.push_back(key[i]);
  }

  ctrl.push_back(key.back());
  ctrl.push_back(key.back());
  ctrl.push_back(key.back());
  return ctrl;
}

Vec2 UniformBspline2D::lookaheadPoint(const std::vector<Vec2>& path, const Vec2& current, double lookahead_dist)
{
  if (path.empty()) return current;
  if (path.size() == 1) return path.front();

  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = dist(path[i], current);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }

  double acc = 0.0;
  for (std::size_t i = nearest + 1; i < path.size(); ++i) {
    const double seg = dist(path[i - 1], path[i]);
    if (acc + seg >= lookahead_dist) {
      const double u = (lookahead_dist - acc) / std::max(1e-9, seg);
      return path[i - 1] + (path[i] - path[i - 1]) * u;
    }
    acc += seg;
  }
  return path.back();
}

}  // namespace ego_2d_planner_pkg
