#pragma once

#include <cstddef>
#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg {
struct GridSpec { std::size_t rows{0}; std::size_t cols{0}; double cell_size_m{1.0}; Vec3 origin_enu; };
Vec3 gridCellCenter(const GridSpec &grid, std::size_t row, std::size_t col);
}
