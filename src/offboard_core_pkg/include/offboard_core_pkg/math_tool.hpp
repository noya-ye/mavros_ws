#pragma once

#include <cmath>
#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg::math_tool {
inline double distance3d(const Vec3 &a, const Vec3 &b) {
  const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
inline double wrapYaw(double yaw) { return std::atan2(std::sin(yaw), std::cos(yaw)); }
}
