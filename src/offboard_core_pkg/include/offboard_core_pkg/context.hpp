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
  double yaw_setpoint_enu{0.0};
  bool publish_position_setpoint{true};

  bool command_pending{false};
  bool command_accepted{false};
  std::string fault;

  bool finitePosition() const {
    return std::isfinite(position_enu.x) && std::isfinite(position_enu.y) &&
           std::isfinite(position_enu.z);
  }
};

}  // namespace offboard_core_pkg
