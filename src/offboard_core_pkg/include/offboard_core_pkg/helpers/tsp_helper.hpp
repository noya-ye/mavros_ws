#pragma once
#include "offboard_core_pkg/context.hpp"
namespace offboard_core_pkg::tsp_helper { inline void setPosition(Context &ctx, const Vec3 &target, double yaw) { ctx.position_setpoint_enu = target; ctx.yaw_setpoint_enu = yaw; } }
