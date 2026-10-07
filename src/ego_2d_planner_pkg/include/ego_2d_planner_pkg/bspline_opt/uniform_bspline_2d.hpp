#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "ego_2d_planner_pkg/common/types.hpp"

namespace ego_2d_planner_pkg
{

class UniformBspline2D
{
public:
  UniformBspline2D() = default;
  UniformBspline2D(const std::vector<Vec2>& ctrl_pts, double knot_span);

  void setControlPoints(const std::vector<Vec2>& ctrl_pts, double knot_span);
  Vec2 evaluate(double t) const;
  std::vector<Vec2> sample(double step) const;

  double duration() const;
  const std::vector<Vec2>& controlPoints() const { return ctrl_pts_; }

  static std::vector<Vec2> resamplePolyline(const std::vector<Vec2>& path, double step);
  static std::vector<Vec2> makeClampedControlPoints(const std::vector<Vec2>& path, double ctrl_dist);
  static Vec2 lookaheadPoint(const std::vector<Vec2>& path, const Vec2& current, double lookahead_dist);

private:
  std::vector<Vec2> ctrl_pts_;
  double knot_span_{0.25};
};

}  // namespace ego_2d_planner_pkg
