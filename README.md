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
publish that altitude. On entry, `land` captures the current valid local XY and
holds it while descending to the landing-approach height, then waits for MAVROS
to report disarmed after a landing request.

To insert downward-camera alignment between hover and land, set
`align_down.enabled: true` for `offboard_core_node`. The camera node publishes
`geometry_msgs/Point` offsets on `/target/circle_center` (preferred) and
`/target/contour_center` (fallback). Configure
`align_down.pixels_per_meter` (calibrated pixels per meter),
`align_down.stable_frames` (consecutive fresh detections),
`align_down.arrive_distance_m` (horizontal error in meters), and
`align_down.max_step_m` (maximum horizontal correction per fresh frame, default
`0.10` m, finite and positive); topic names are
overridable via `align_down.circle_topic` and `align_down.contour_topic`.
The image's up and left directions must correspond to vehicle forward and
left; confirm camera orientation, offset signs, and pixel scale in SITL
before enabling flight. Alignment holds the entry altitude and commands the
home yaw; body offsets are rotated using the measured current yaw. Lost
detections hold the current XY at the entry altitude. Alignment
does not have a timeout and therefore waits for detections before landing.

## YOLO Detection Topic

`yolov8_seg_usb.py` publishes each frame's detections to `/yolo/detections`
as `std_msgs/Float32MultiArray`. The data is a flat sequence of alternating
`class_id, confidence` values; an empty array means no detections in that
frame. The topic can be changed with `--topic`. The `offboard_core_node`
subscriber stores received pairs in `Context::yolo_detections` as
`YoloDetection { class_id, confidence }`, with a receive timestamp and
sequence counter. Its topic can be changed with the `yolo.detections_topic`
parameter. Source ROS 2 Humble before running the script so `rclpy` is available.

```bash
source /opt/ros/humble/setup.bash
python3 yolov8_seg_usb.py --engine best.engine
```

Inspect the stream with `ros2 topic echo /yolo/detections`.

## Build and Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding offboard_core_pkg
source install/setup.bash
ros2 launch offboard_core_pkg offboard_core.launch.py
```

Snake coverage with EGO obstacle avoidance and down-camera alignment runs with:

```bash
ros2 launch offboard_core_pkg snake_ego_avoid.launch.py
```

The same task sequence is also available as a dedicated node whose alignment
calibration matches `offboard_core.yaml`:

```bash
ros2 launch offboard_core_pkg align_drop_snake_ego.launch.py
```

Its snake and EGO avoidance settings follow
`config/snake_ego_avoid.yaml`; alignment settings are in
`config/align_drop_snake_ego.yaml`.

The node prioritizes fresh down-contour detections over obstacle avoidance,
returns to the interrupted position after alignment, and fails if alignment
exceeds `align_down.timeout_s` (default `10.0` seconds). Configure camera topics,
calibration, `align_down.max_step_m`, and detection freshness in `config/snake_ego_avoid.yaml`. After a
successful alignment it suppresses repeat alignment for the same circle center
within `align_down.retrigger_radius_m` (default `0.55` m).

## Flight Recording

Use `record_flight.py` to record selected topics. It does not start or
capture any launch process:

```bash
source /opt/ros/humble/setup.bash
python3 record_flight.py \
  -t /position_cmd \
  -t /fastlio2/lio_odom \
  -t /mavros/state \
  -t /mavros/local_position/pose
```

Press `Ctrl-C` to stop recording. The output directory contains the rosbag
under `bag/`.

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

EGO supplies horizontal motion only. When `EgoGotoTask` begins, it captures
the current MAVROS ENU altitude and holds that value for the entire EGO phase;
the Z position, velocity, and acceleration fields of `/position_cmd` are
ignored. This prevents the 2D planner's nominal Z output from changing flight
altitude.

## Snake EGO Avoidance

The snake/EGO avoidance node can be launched with:

```bash
ros2 launch offboard_core_pkg snake_ego_avoid.launch.py
```

Its parameters are in `config/snake_ego_avoid.yaml`. In particular,
`snake.first_axis` accepts `x_first` or `y_first`, and `snake.max_step_m`
controls the maximum commanded snake-path step per 0.05 s reference interval.
The node captures yaw from its first valid MAVROS local pose and commands that
same heading on every outgoing setpoint, including takeoff, snake traversal,
EGO avoidance, and task transitions. Once PX4 takes over in LAND mode, yaw is
controlled by the flight controller rather than this node.
During operation, `[SNAKE_EGO]` logs report the task phase, current grid cell,
aircraft ENU position, target point, occupied-cell skips, EGO avoidance, and
landing failures. Periodic SNAKE and AVOIDING status lines are throttled to
one per second.

The restricted build environment cannot initialize CycloneDDS because it has no
enumerable UDP interface. Run the node and MAVROS/PX4 SITL validation in a
normal ROS 2 environment with DDS networking available.

## Lidar to PX4 Bridge

`mavros_lidar.launch.py` starts the MID-360 driver, then the `fastlio2`
package's `lio_launch.py`, then the bridge. The FAST-LIO workspace prefix must
be sourced before this workspace:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/fastlio2_ws/install/setup.bash
source /home/jetson/mavros_ws/install/setup.bash
ros2 launch offboard_core_pkg mavros_lidar.launch.py
```

`lidar_to_px4_bridge` forwards `/fastlio2/lio_odom` to
`/mavros/vision_pose/pose`. It rejects non-finite positions and any position
jump larger than `0.1 m` relative to the last accepted sample. The threshold
can be adjusted with the `jump_threshold_m` ROS parameter when needed.
