# MAVROS Offboard Workspace

This workspace uses the task architecture in `offboard_core_pkg`:
`Context`, `MavrosIface`, `Scheduler`, `ITask`, and independent task
implementations. The workspace also contains `door_navigation`, which provides
the door perception and geometric path planner.

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
Fresh circle detections take priority. When circle data is stale, a contour
within 40 pixels of the most recently received circle point is treated as an
equivalent circle detection for arrival checking.
The image's up and left directions must correspond to vehicle forward and
left; confirm camera orientation, offset signs, and pixel scale in SITL
before enabling flight. Alignment holds the entry altitude and commands the
home yaw; body offsets are rotated using the measured current yaw. Lost
detections hold the current XY at the entry altitude. Alignment
does not have a timeout and therefore waits for detections before landing.
An optional single-topic red-cross alignment can run after AlignDown and before
landing by setting `red_cross_align.enabled: true`; it consumes
`/target/red_cross_center` (configurable with `red_cross_align.topic`) and
reuses the same pixel calibration, stability, arrival, and step parameters.
`camera_center_node` publishes this topic when a red-cross detection is present.
At the first fresh circle/contour frame within the arrival tolerance, it checks
YOLO once: a detection received within 0.5 seconds with confidence strictly
greater than 0.40 prints `FIND TARGET` and allows the stable-frame check to
continue. Missing, stale, empty, or insufficient-confidence YOLO data prints
`NO TARGET` and returns `SUCCESS`, allowing the scheduler to advance to
the next task without waiting for stable frames. The target check is reset by
`onEnter`, and is retained across pause/resume and later tolerance crossings.
`offboard_core_node` and both snake/EGO node executables subscribe to
`/yolo/detections` by default. The topic can be changed with the
`yolo.detections_topic` parameter in each node's configuration.

## YOLO Detection Topic

`yolov8_seg_usb.py` publishes each frame's detections to `/yolo/detections`
as `std_msgs/Float32MultiArray`. The data is a flat sequence of alternating
`class_id, confidence` values; an empty array means no detections in that
frame. The topic can be changed with `--topic`. The `offboard_core_node`
subscriber stores received pairs in `Context::yolo_detections` as
`YoloDetection { class_id, confidence }`, with a receive timestamp and
sequence counter. Each node's topic can be changed with the
`yolo.detections_topic` parameter. Source ROS 2 Humble before running the script
so `rclpy` is available.

```bash
source /opt/ros/humble/setup.bash
python3 yolov8_seg_usb.py --engine best.engine
```

The script subscribes to `/target/debug_image` by default. Start
`camera_center.launch.py` with `publish_debug: true` so the camera node is the
only process that opens `/dev/video0`. Use `--image-topic` to select another
`sensor_msgs/Image` topic.

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

When an occupied cell is near the segment to the current snake waypoint,
the task selects the first unoccupied waypoint from the remaining route,
sends it to EGO and enters EGO avoidance directly. Segment detection uses
`avoidance.trigger_distance_m` plus the grid-resolution margin; there is no
speed-dependent braking distance, intermediate braking setpoint or distance
handoff stage. Candidate targets are checked at their own grid cells without
an additional path collision check. EGO emergency stop remains enabled.

The node checks fresh down-circle and down-contour detections, as well as
RedCross detections when enabled, before obstacle avoidance. RedCross has
priority when both target types are fresh; an active alignment is not
interrupted by another target. On alignment timeout, it returns to the saved
pre-alignment position and resumes the interrupted route. Configure camera
topics, calibration, `align_down.max_step_m`, detection freshness, and
`align_down.retrigger_radius_m` in
`config/align_drop_snake_ego.yaml`. After successful alignments, the node keeps
all completed target positions for the current task run. New contour detections
are projected into ENU and skipped when they fall within the configured radius
(default `0.55` m) of any completed target. Circle position is preferred when
recording a completed target; fresh contour position is the fallback, followed
by the aircraft position at arrival tolerance. A later `NO TARGET` result does
not discard the saved position.
For down-target suppression, both fresh circle and contour projections are
checked; either projection inside the radius suppresses a new alignment.

After a down alignment succeeds, `align_down.trigger_cooldown_s` (default
`2.0` s) delays the next down alignment trigger. The interval starts when the
alignment task succeeds, so time spent aligning does not count toward it. Set
the parameter to `0.0` to disable this cooldown.

Both down and RedCross alignment abandon an attempt after detections remain
stale for `align_down.loss_timeout_s` (default `1.0` s), return to the saved
entry pose, and resume the route. With the `0.5` s detection freshness window,
this is approximately `1.5` s after the last detection. A fresh detection for
the active alignment source resets this loss timer; another source does not.
For down alignment, the entry target estimate is suppressed within `align_down.retrigger_radius_m`
for `align_down.loss_retry_cooldown_s` (default `3.0` s) of resumed route time,
then becomes eligible again. This does not mark an abandoned target completed.
The total alignment timeout still applies while detections continue, and loss
handling does not interrupt a drop already in progress. Restart the node after
changing these startup parameters.

`align_drop_snake_ego_node` can also align `/target/red_cross_center` when
`red_cross_align.enabled` is true. Its pixel scale, stable-frame count, arrival
tolerance, step limit, and timeout are independent from `align_down.*`. If
RedCross and down circle/contour detections are both fresh when an alignment
begins, RedCross is selected; an active calibration is not interrupted by the
other target.
RedCross has one target per task run: after its first successful alignment,
further RedCross triggers are disabled until a new task run or node restart.
Before that success, fresh RedCross detections can trigger regardless of
circle/contour completed-target radii or loss cooldowns. A timeout or detection
loss does not count as success, so RedCross can retry after returning. RedCross
positions are never added to the circle/contour suppression list.
Successful RedCross calibration, or AlignDown success with a confirmed YOLO
target, runs `DownDropTask`; on success the task returns to the saved pose and
resumes snake coverage or EGO avoidance. The three DownDrop attempts use IDs
0, 1, and 2 with their configured offsets in order. After ID 2 completes and
the aircraft returns to the saved pose, the combined task finishes. Configure
these values in `config/align_drop_snake_ego.yaml` under `red_cross_align.*`
and `down_drop.*`.

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
and `ego.yaw_align_rad=0`. `EgoGotoTask` captures MAVROS yaw when the EGO task
starts and holds that yaw through planning, arrival, and task exit.

`ego.kp_xy` in `config/ego_test.yaml` sets the horizontal EGO position-error
correction gain for `ego_test_node` (default `1.0`). The planner caps the error
at 0.30 m before multiplying by this gain. Edit the YAML and restart the node
to apply a new value; this parameter does not update the active planner at runtime.

## Door Navigation Flight

`door_navigation.launch.py` starts the door planner and
`door_navigation_node`. The offboard sequence is
`presetpoint -> set_offboard -> arm -> takeoff -> hover -> door navigation ->
land`. The task reuses the EGO2D interfaces from `ego_test_node`: it publishes
goals to `/simple_2d_planner/goal`, reads `/position_cmd` and
`/fastlio2/lio_odom`, and runs `EgoVelPlanner` to produce MAVROS setpoints.
For each approach/crossing phase it follows the selected waypoint from
`/door/path` only while `/door/status` is fresh, reports a geometric reference,
and marks the current segment clear. It holds position when those checks fail
and advances to landing after the door planner reports two completed crossings.

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/ego_planner_ws/install/setup.bash
source install/setup.bash
ros2 launch offboard_core_pkg door_navigation.launch.py
```

The launch starts the door planner in geometric preview mode so it does not
publish its own timed trajectory; EGO2D remains the flight trajectory
controller. FAST-LIO, the EGO2D planner publishing `/position_cmd`, and MAVROS
must already be running. Configure `door.planning_frame` and the `ego.*` axis
mapping in `config/door_navigation.yaml` to match those nodes. This launch has
`auto_start: true` and begins takeoff/control tasks when MAVROS localization is
available; validate it in SITL before flight.

## Corridor Door Flight

`corridor_door.launch.py` starts the occupancy-grid corridor task. It scans
forward along ENU +x for a free run bounded by occupied cells, sends the run
midpoint to EGO, then moves another 0.1 m along +x to count the door as
crossed. Set `corridor.door_count` in `config/corridor_door.yaml` to choose
the number of doors. Unknown grid cells are not treated as free space.

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/ego_planner_ws/install/setup.bash
source install/setup.bash
ros2 launch offboard_core_pkg corridor_door.launch.py
```

The launch expects MAVROS, FAST-LIO, and EGO2D (including
`/ego_2d_planner/occupancy_grid`, `/position_cmd`, and `/fastlio2/lio_odom`)
to be running. A candidate opening must be between
`corridor.min_opening_width_m` and `corridor.max_opening_width_m` (defaults
0.5 m and 1.5 m). Configure these limits, the forward scan range, occupancy
freshness, and EGO frame mapping in `config/corridor_door.yaml`; validate it
in SITL before flight.

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
