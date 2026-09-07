# MAVROS Offboard Workspace

This workspace was rebuilt around the task architecture used by
`offboard_core_pkg`: `Context`, `MavrosIface`, `Scheduler`, `ITask`, and
independent task implementations. It intentionally has one source package,
`offboard_core_pkg`, rather than the prior manager/modules/watchdog split.

Its source tree mirrors the reference layout: `include/offboard_core_pkg` holds
context, scheduler, helpers, planners, tasks, and the MAVROS interface;
`src/nodes`, `src/tasks`, `src/planners`, and `src/utils` hold the corresponding
implementation areas. The task tree contains only MAVROS-backed core tasks.

## MAVROS and PX4 DDS boundary

The reference package sends PX4 DDS messages directly: `OffboardControlMode`,
`TrajectorySetpoint`, and `VehicleCommand`, in PX4 NED conventions. This
workspace must not publish those topics. `MavrosIface` instead:

- reads `/mavros/state`, local pose, and local velocity;
- continuously publishes `geometry_msgs/PoseStamped` to
  `/mavros/setpoint_position/local` in ROS ENU;
- requests `OFFBOARD`, arming, and landing with MAVROS services.

MAVROS performs the ENU/NED conversion. A successful service response only
means MAVROS accepted the request; tasks wait for `/mavros/state` to report the
actual mode or armed state. The scheduler sends setpoints before requesting
`OFFBOARD`, because PX4 requires a valid setpoint stream before accepting it.

## Default Task Sequence

The node schedules six independent tasks in this order:
`presetpoint -> set_offboard -> arm -> takeoff -> hover -> land`.

`presetpoint` waits for MAVROS connection and a valid local pose, captures the
current ENU pose/yaw, and publishes that target for the configured warm-up
period. The command tasks retry unavailable or rejected services until their
timeouts, while confirming the resulting mode/arming state from `/mavros/state`.
`takeoff` raises the captured ENU setpoint by `takeoff_height_m` and waits for
the measured altitude to enter `takeoff_tolerance_m`. `hover` then continues to
publish that altitude, and `land` waits for MAVROS to report disarmed after a
landing request.

## Build and Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding offboard_core_pkg
source install/setup.bash
ros2 launch offboard_core_pkg offboard_core.launch.py
```

`auto_start` defaults to `false`. Validate MAVROS connection, the local frame,
and takeoff direction in SITL before setting it to `true`.

The task timing parameters are `presetpoint_duration_s`, `command_timeout_s`,
`command_retry_interval_s`, `takeoff_height_m`, `takeoff_tolerance_m`,
`takeoff_timeout_s`, `hover_duration_s`, and `land_timeout_s` in
`config/ego_goto.yaml`.

## EGO Test Flight

`ego_test_node` runs `presetpoint -> set_offboard -> arm -> takeoff 1 m ->
hover 5 s -> EGO +x 2 m -> land`. It publishes the EGO goal to
`/simple_2d_planner/goal`, consumes FAST-LIO odometry and `/position_cmd`, and
uses `auto_start: true`; launching it begins the sequence after MAVROS has a
valid local pose.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch offboard_core_pkg ego_test.launch.py
```

The defaults in `config/ego_test.yaml` require FAST-LIO and MAVROS local
coordinates to remain aligned in ENU: `ego.swap_xy=false`, axis signs of `1`,
and `ego.yaw_align_rad=0`.

The restricted build environment cannot initialize CycloneDDS because it has no
enumerable UDP interface. Run the node and MAVROS/PX4 SITL validation in a
normal ROS 2 environment with DDS networking available.

## Lidar to PX4 Bridge

`lidar_to_px4_bridge` forwards `/fastlio2/lio_odom` to
`/mavros/vision_pose/pose`. It rejects non-finite positions and any position
jump larger than `0.1 m` relative to the last accepted sample. The threshold
can be adjusted with the `jump_threshold_m` ROS parameter when needed.
