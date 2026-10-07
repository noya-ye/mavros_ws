#pragma once

#include <array>
#include <vector>
#include "ego_2d_planner_pkg/common/types.hpp"

namespace ego_2d_planner_pkg
{
class GridMap2D;

class PolynomialTrajectory2D
{
public:
  struct Piece
  {
    double duration{0.0};
    std::array<Vec2, 6> coefficients{};  // ascending powers of local time
  };

  std::vector<Piece> pieces;
  bool valid() const;
  double duration() const;
  Vec2 evaluate(double time, int derivative = 0) const;
  std::vector<Vec2> sample(double time_step, double from = 0.0) const;
  // Conservative Bezier bounds cover the whole continuous curve, including between samples.
  bool collisionFree(const GridMap2D& map, double from = 0.0) const;
  bool feasible(double max_vel, double max_acc) const;
};

Vec2 lookaheadPoint2D(const std::vector<Vec2>& path, const Vec2& position, double distance);
}  // namespace ego_2d_planner_pkg
