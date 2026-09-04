#pragma once
#include <vector>
#include "offboard_core_pkg/context.hpp"
namespace offboard_core_pkg { class TspMinSnapPlanner { public: std::vector<Vec3> order(const std::vector<Vec3> &points) const; }; }
