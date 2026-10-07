# EGO Turn Test Analysis

Source: `/home/jetson/mavros_ws/ego_turn_test/ego_turn_test_0.db3`.
All times below are seconds since the first recorded message. The original bag is opened read-only.
No flight configuration or PX4 parameter was changed.

## Findings

- Bag duration: 75.37 s. EGO tracking with raw setpoints: 29.12-61.82 s.
- Position tracking error (EGO command versus interpolated received FAST-LIO position): median 0.170 m, P95 0.505 m, maximum 0.538 m at 52.32 s.
- First turn: actual Y minimum -0.855 m versus commanded minimum -0.487 m in the 36-44 s window. Maximum tracking error 0.374 m.
- Second turn: actual Y maximum 0.995 m versus commanded maximum 0.477 m in the 46-56 s window. Maximum tracking error 0.538 m.
- At 39.57 s, actual Y velocity was -0.511 m/s; EGO Y velocity was -0.188 m/s and Y acceleration was +0.088 m/s2 (already braking).
- At 50.17 s, actual Y velocity was +0.517 m/s; EGO Y velocity was +0.169 m/s and Y acceleration was -0.109 m/s2. The raw position correction was already -0.100 m in Y.
- Raw velocity feedforward peak: 0.360 m/s; raw acceleration feedforward peak: 0.494 m/s2. Neither reaches the current code's 0.5 m/s or 0.6 m/s2 caps. Causal sample comparisons and position mapping residuals confirm the position correction uses the latest received EGO command and FAST-LIO odometry, with unit gain and a 0.30 m cap.
- The position correction reaches the 0.30 m cap; 17.6% of interpolated tracking errors exceed 0.30 m. Increasing this cap is not justified by this test: it would permit more aggressive corrective commands.
- EGO publishes at 50 Hz; raw setpoints at 20 Hz, maximum gap 53 ms. No significant setpoint interruption during tracking.
- Trajectory replacements have adjacent command position steps <= 5.3 mm and velocity steps <= 0.0046 m/s. These differences include ordinary motion over the 20 ms command interval; no large replacement discontinuity is visible.
- FAST-LIO publishes at 10 Hz. Median message age at bag receipt is 81 ms. The latest received LIO sample used at raw setpoint times is approximately 135 ms old (P95 187 ms). This can add phase lag to the external position correction.
- Active-flight estimator samples (33): horizontal velocity and relative horizontal position flags valid throughout; no GPS glitch or accelerometer error flags. This does not prove all estimator internals are correct.
- Position-only goal hold starts at 61.87 s. In the remaining pre-landing interval through 63.3 s, PX4 X position exceeds the held goal by only 0.011 m. Large overshoot is principally at the turns in this recording.

## Interpretation and Limits

The data shows lateral overshoot and delayed braking in the combined external position correction and PX4 control system. Raw velocity is a feedforward term: PX4 also adds its own position feedback, so actual speed exceeding the raw velocity is not by itself evidence of a faulty velocity controller. The recorded position correction and acceleration already oppose the second turn near its velocity peak, which supports investigating response and damping.

The bag contains no PX4 ULog, internal velocity setpoints, thrust/attitude setpoints, or actual PX4 parameter snapshot. It cannot distinguish velocity loop damping, position loop gain, thrust model error, and actuator response conclusively. Current source/config values are reference values, not a recorded parameter snapshot. No specific PX4 gain multiplier can be established from this bag alone.

## Proposed Experiments

1. Establish a slower planning baseline: `optimization/max_vel: 0.25` and `optimization/max_acc: 0.35` in the planner YAML actually passed to `cloud_ego.launch.py`. Current source defaults are 0.35 and 0.8. Keep velocity and acceleration feedforward scales at 1.0 initially. Re-record the same route and compare turn errors and braking.
2. The planner's feasibility limits are soft costs, not guaranteed bounds. Check the resulting `/position_cmd` speed/acceleration before treating the requested limits as achieved. If still too high, increase `optimization/knot_span` from 0.60 to 0.80 as a separate experiment, then verify actual commands and collision checks. Increasing span scales time for fixed control points, but optimization and frequent replanning can change those points.
3. Increase `ego_test_node`'s `setpoint_rate_hz` from 20 to 50 as a separate experiment. This reduces command sampling delay but cannot refresh the 10 Hz LIO samples or guarantee correction of the observed overshoot.
4. Inspect PX4 ULog and export current parameters. Compare internal velocity setpoint, actual velocity, attitude setpoint/actual attitude, thrust, and integrator behavior around 38-42 s and 48-54 s. If velocity tracking itself overshoots without attitude/thrust saturation, tune the velocity loop damping first; if internal velocity setpoints are themselves too aggressive, inspect position gain and external correction.
5. If needed, expose `EgoVelPlanner::Config::kp_xy` as a ROS parameter, then trial a reduction from 1.0 to 0.7-0.8 while leaving the 0.30 m cap unchanged. This is an experiment to reduce the external position correction; it can increase lag. It is not currently configurable through `ego_test.yaml`, and lowering it is not a confirmed fix.
6. Investigate LIO latency and timestamp meaning. Compare measurements at corresponding header times before diagnosing frame offsets. If introducing prediction, validate the velocity frame and timestamp first; merely publishing repeated old odometry faster does not remove latency.

Use one experiment at a time after establishing the slower baseline. A useful initial comparison target is maximum turn tracking error below 0.25 m, lower actual lateral speed peaks, and no additional oscillation; this is an evaluation target, not a guaranteed outcome.

## Artifacts

- `tracking.png`: trajectory, position error, velocity, acceleration feedforward and trajectory IDs.
- `metrics.json`: machine-readable metrics, mode changes, estimator flags and sampled timeline.
- `*.csv`: decoded topic data; columns follow the extraction code in `analyze.py`.
- `analyze.py`: repeatable offline analysis. Source ROS Humble and this workspace before running it with `/usr/bin/python3`.
