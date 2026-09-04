#include "offboard_core_pkg/grid.hpp"
namespace offboard_core_pkg {
Vec3 gridCellCenter(const GridSpec &grid, std::size_t row, std::size_t col) {
  return {grid.origin_enu.x + (static_cast<double>(col) + 0.5) * grid.cell_size_m,
          grid.origin_enu.y + (static_cast<double>(row) + 0.5) * grid.cell_size_m,
          grid.origin_enu.z};
}
}
