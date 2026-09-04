#pragma once
#include <vector>
#include "offboard_core_pkg/context.hpp"
namespace offboard_core_pkg { class MinSnapPlanner { public: std::vector<Vec3> plan(const std::vector<Vec3> &waypoints) const; }; }
