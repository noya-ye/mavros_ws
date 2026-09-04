#pragma once
#include <vector>
#include "offboard_core_pkg/context.hpp"
namespace offboard_core_pkg { class AStarPlanner { public: std::vector<Vec3> plan(const Vec3 &start, const Vec3 &goal) const { return {start, goal}; } }; }
