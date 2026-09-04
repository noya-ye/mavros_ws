#pragma once
#include "offboard_core_pkg/math_tool.hpp"
namespace offboard_core_pkg::yaw_assist { inline double hold(double yaw) { return math_tool::wrapYaw(yaw); } }
