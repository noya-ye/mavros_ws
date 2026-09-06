#pragma once

#include <cmath>
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

  bool finitePosition() const {
    return std::isfinite(position_enu.x) && std::isfinite(position_enu.y) &&
           std::isfinite(position_enu.z);
  }
};

}  // namespace offboard_core_pkg
