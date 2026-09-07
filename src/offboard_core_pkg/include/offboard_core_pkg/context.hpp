#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <string>

namespace offboard_core_pkg {

// MAVROS local-position and setpoint APIs use ROS ENU coordinates. MAVROS
// performs the ENU <-> PX4 NED conversion at the bridge boundary.
struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class SetpointMode {
  POSITION,
  POSITION_VELOCITY,
  POSITION_VELOCITY_ACCELERATION,
  VELOCITY_ONLY,
};

struct Context {
  bool connected{false};
  bool armed{false};
  bool position_valid{false};
  std::string mode;
  Vec3 position_enu;
  Vec3 velocity_enu;
  double yaw_enu{0.0};

  bool home_initialized{false};
  Vec3 home_enu;
  double home_yaw_enu{0.0};

  // The scheduler writes this every cycle; the interface publishes it at the
  // configured rate, including during MAVROS OFFBOARD warm-up.
  Vec3 position_setpoint_enu;
  // Optional raw setpoint feed-forward terms, expressed in ROS ENU.
  Vec3 velocity_setpoint_enu;
  Vec3 acceleration_setpoint_enu;
  double yaw_setpoint_enu{0.0};
  double yaw_rate_setpoint_enu{0.0};
  bool publish_position_setpoint{true};
  SetpointMode setpoint_mode{SetpointMode::POSITION};
  bool use_position_velocity_acceleration{false};

  bool command_pending{false};
  bool command_accepted{false};
  std::string fault;

  // EGO planner input state (camera/odometry frame).
  bool ego_cmd_valid{false};
  std::uint64_t ego_cmd_stamp_us{0};
  Vec3 ego_cmd_position;
  Vec3 ego_cmd_velocity;
  Vec3 ego_cmd_acceleration;
  double ego_cmd_yaw{0.0};
  bool ego_odom_valid{false};
  std::uint64_t ego_odom_stamp_us{0};
  Vec3 ego_odom_position;
  Vec3 ego_odom_velocity;

  // Inflated occupancy grid published by ego_2d_planner_pkg.
  // The grid is expressed in the same local frame as position_enu.
  bool occupancy_grid_valid{false};
  std::uint64_t occupancy_grid_stamp_us{0};
  double occupancy_grid_resolution{0.0};
  double occupancy_grid_origin_x{0.0};
  double occupancy_grid_origin_y{0.0};
  std::uint32_t occupancy_grid_width{0};
  std::uint32_t occupancy_grid_height{0};
  std::vector<std::int8_t> occupancy_grid_data;

  bool finitePosition() const {
    return std::isfinite(position_enu.x) && std::isfinite(position_enu.y) &&
           std::isfinite(position_enu.z);
  }
};

}  // namespace offboard_core_pkg
