# Build Progress

## Baseline

- Date: 2026-09-04
- Workspace: `/home/jetson/mavros_ws`
- ROS baseline: ROS 2 Humble, C++17, ament_cmake, MAVROS 2.x.
- Source package: `offboard_core_pkg`.
- Design: single-package `Context + MavrosIface + Scheduler + ITask + Tasks` architecture.

## Verification

- `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select offboard_core_pkg --allow-overriding offboard_core_pkg --event-handlers console_direct+`
  completed successfully: `1 package finished` after reorganizing the source tree.
- `source /opt/ros/humble/setup.bash && source install/setup.bash && ROS_LOG_DIR=/tmp/mavros_ws_roslog ros2 launch offboard_core_pkg offboard_core.launch.py --show-args`
  parsed successfully.
- After removing all unimplemented task placeholders, `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select offboard_core_pkg --allow-overriding offboard_core_pkg --event-handlers console_direct+` completed successfully: `1 package finished`; launch argument parsing passed again.
- After aligning `ITask` and `Scheduler` with the reference lifecycle and interrupt model, the same build command completed successfully and `ros2 launch offboard_core_pkg offboard_core.launch.py --show-args` parsed successfully. The default MAVROS task implementation was restored after its source directory was found empty during this change.
- Implemented independent `PresetpointTask`, `ArmTask`, `SetOffboardTask`, `HoverTask`, and `LandTask` source/header pairs. `offboard_core_node` now schedules `presetpoint -> set_offboard -> arm -> hover -> land`; the focused build completed successfully and the launch file parsed with no arguments.
- After the final node-order adjustment, the focused build completed successfully: `1 package finished [7.64s]`. A default, non-commanding `ros2 run offboard_core_pkg offboard_core_node` smoke test could not initialize CycloneDDS because the restricted environment could not enumerate UDP interfaces (`rmw_create_node: failed to create domain`); no node-liveness or MAVROS behavior was verified.
- Added `TakeoffTask` and completed the node sequence as `presetpoint -> set_offboard -> arm -> takeoff -> hover -> land`; validation is pending the build below.
- `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select offboard_core_pkg --allow-overriding offboard_core_pkg --event-handlers console_direct+` completed successfully: `1 package finished [10.6s]`.
- `source /opt/ros/humble/setup.bash && source install/setup.bash && ROS_LOG_DIR=/tmp/mavros_ws_roslog ros2 launch offboard_core_pkg offboard_core.launch.py --show-args` parsed successfully with no arguments.
- Date: 2026-09-07. Added `ego_test_node`: `presetpoint -> set_offboard -> arm -> takeoff(1 m) -> hover(5 s) -> EGO +x(2 m) -> land`. The test defaults to direct ENU mapping (`swap_xy=false`, unity axis signs, zero yaw alignment) because FAST-LIO and MAVROS local coordinates were verified aligned. `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select offboard_core_pkg --allow-overriding offboard_core_pkg --event-handlers console_direct+` completed successfully: `1 package finished [0.98s]`. `source /opt/ros/humble/setup.bash && source install/setup.bash && ROS_LOG_DIR=/tmp/mavros_ws_roslog ros2 launch offboard_core_pkg ego_test.launch.py --show-args` parsed successfully. No MAVROS/PX4 flight behavior was executed.
- Date: 2026-09-07. Added position jump protection to `lidar_to_px4_bridge`: non-finite positions and jumps greater than the default `0.1 m` are rejected before publishing, and rejected samples do not update the accepted-position baseline. `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select offboard_core_pkg --allow-overriding offboard_core_pkg --event-handlers console_direct+` completed successfully: `1 package finished [1min 1s]`. No MAVROS/PX4 runtime behavior was executed.
- `source /opt/ros/humble/setup.bash && source install/setup.bash && ROS_LOG_DIR=/tmp/mavros_ws_roslog ros2 launch offboard_core_pkg mavros_lidar.launch.py --show-args` parsed successfully with no arguments; the same check without `ROS_LOG_DIR` was blocked by the read-only default `/home/jetson/.ros/log` path.

## Known Limits

- No MAVROS, PX4 SITL, DDS network, or real-flight behavior has been validated.
- In this sandbox, CycloneDDS cannot enumerate UDP interfaces, so run node/SITL validation in an environment with available DDS network interfaces.
- `auto_start` is false by default; do not enable it until local-frame and MAVROS service behavior are validated in SITL.
- `TakeoffTask` commands a relative ENU altitude increase from the presetpoint and requires valid position feedback plus armed state before completing.
- `/home/jetson/ros2_ws_px4/install` exports a DDS package with the same package name. Build with `--allow-overriding offboard_core_pkg`, and source this workspace last when running it.

## Checklist

- [x] Compile the package.
- [x] Parse the launch file.
- [x] Compile the independent five-task offboard sequence.
- [x] Compile the complete six-task flight sequence.
- [x] Compile and parse the `ego_test_node` launch entry.
- [ ] Verify the node remains alive with MAVROS/SITL.
- [ ] Validate OFFBOARD, arming, takeoff, and landing behavior in SITL.
