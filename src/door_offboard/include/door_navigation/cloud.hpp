#pragma once
#include "door_navigation/core.hpp"
#include <cstdint>

namespace door_navigation {
// PointCloud2 layout without a ROS dependency, so byte parsing can be tested offline.
struct CloudField { uint32_t offset=0; uint8_t datatype=7; };
struct CloudView {
  const uint8_t *data=nullptr;
  size_t size=0;
  uint32_t width=0, height=0, point_step=0, row_step=0;
  bool big_endian=false;
  std::array<CloudField,3> xyz;
};
std::vector<Vec3> read_cloud(const CloudView &cloud, const Vec3 &body, double self_radius);
// Fill only narrow angular gaps between measured rays, up to the nearer return.
// Existing obstacle cells are never cleared; unmeasured wide sectors stay unknown.
void observe_scan(Grid &grid, const Route &points, const Vec2 &origin);
// Close only one-cell raster sampling holes; preserve obstacles and large shadows.
void complete_observation(Grid &grid);
// Fixed corridor model: fill visible space between measured side walls, retaining
// shadows behind reconstructed wall/door segments. -1 means no wall pair found.
int complete_corridor_observation(Grid &grid, const Vec2 &origin);
}  // namespace door_navigation
