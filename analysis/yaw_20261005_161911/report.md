# 2026-10-05 16:19:11 飞行末段 yaw 分析

## 结论

末段不是单纯的 Euler yaw ±180° 显示跳变，而是 FAST-LIO 四元数所描述的连续大幅旋转：16:21:41–16:21:45，展开后的 yaw 从约 23° 增至约 362°，峰值约 250.5°/s。之后落地，yaw 稳定在约 −14°，并非直到记录结束仍持续乱飘。

同一次飞行的 MAVROS 日志明确记录了低电量和严重低电量事件。电池压降导致动力/偏航控制余量不足，是优先排查假设；目前只能确定时间相关性，不能证明因果。现有数据不能排除外部航向估计异常、磁航向融合冲突、执行器异常或其他飞控控制问题。

## 数据范围

- Bag：`rosbag2_2026_10_05-16_19_11/rosbag2_2026_10_05-16_19_11_0.db3`。
- 时间：北京时间 16:19:11.465–16:22:08.116，约 176.65 秒。
- FAST-LIO odometry：1767 条，约 10 Hz。
- 同时记录：LIO path、规划路径/占据栅格、3 条规划 goal、circle/contour/red-cross 图像中心。
- 未记录：MAVROS local pose、raw setpoint、state、battery、IMU、vision pose、estimator status、原始 LiDAR。
- 补充使用本机 `.ros/log` 中本次 MAVROS、控制任务、FAST-LIO、bridge 日志。未回放 bag，未向飞机发送命令，也未修改飞行代码或参数。

## 时间线

| 北京时间 | 证据 | 含义 |
| --- | --- | --- |
| 16:21:14.718 | MAVROS EVENT 5049764：`check_battery_low` | 飞控已报低电量 |
| 16:21:14.732 | EVENT 12216659：`Low battery level, return advised` | 飞控提示返航；此后任务仍继续 |
| 16:21:39.471 | 控制任务开始下一段 EGO，目标约 (3.91, 4.94, 1.12) | yaw 大幅变化前的任务切换 |
| 16:21:41.521 | LIO yaw 22.96° | 明显偏离之前接近 0° 的航向 |
| 16:21:42.401 | LIO yaw 45.61° | 航向继续加速变化 |
| 16:21:42.993 | EVENT 2627595：`check_battery_critical` | 飞控报严重低电量 |
| 16:21:43.008 | EVENT 2474944：`Critical battery level, land now` | 飞控提示立即降落；这条事件本身是 Warn 动作，不能推断已经自动切 LAND |
| 16:21:43.115 | LIO yaw 91.95° | 此时还未触发末次视觉对齐 |
| 16:21:44.020 | 控制日志保存返回姿态：MAVROS yaw −2.047 rad（−117.28°） | 飞控姿态输出也已大幅偏转 |
| 16:21:44.021 | 触发 AlignDown；目标 yaw −0.007 rad（−0.40°） | 对齐任务在异常之后触发，不是这次偏转的起点 |
| 16:21:44.216 | LIO yaw 277.47°；约 0.100 s 内增 25.08° | 峰值 yaw 变化率约 250.5°/s |
| 16:21:45.020 | 控制日志 MAVROS yaw +0.058 rad（+3.32°） | 与 LIO 回到约 2° 的表现一致，展开后约为 362° |
| 16:21:46.421 | 对齐检测超时，保持位置 | 视觉对齐仍在执行 |
| 16:21:48.594 | EVENT 9844908：`Landing detected` | 飞控检测落地 |
| 16:21:50.437 | 控制节点收到 SIGINT/SIGTERM | 控制进程停止 |
| 16:21:50.596 | EVENT 16017271：`Disarmed by` | 飞控解除解锁 |

事件 ID 使用本机 PX4 源码的 FNV-1a 24 位映射解码，结果和源码位置保存在 `decoded_events.json`。本机源码是否与飞机固件完全一致未验证；上述事件名称均通过精确 ID 匹配获得，未对未知事件做猜测。

## 排除与限制

1. **不是单纯角度包络问题。** 对四元数归一化后计算 roll/pitch/yaw，并对 yaw 做 unwrap，仍得到接近一圈的连续变化。四元数模长误差最大 2.45e−15。异常期间 pitch 在约 −17° 到 +13°，没有接近 Euler 奇异点 ±90°。
2. **不像 bag 丢帧或时间倒退造成。** LIO header 相邻时间间隔为 0.0931–0.1083 秒，持续约 10 Hz；接收时刻相对 header 延迟为 0.0347–0.2341 秒。MAVROS 的 Time jump 事件在 16:16:12，早于本次 bag，不能当作末段直接原因。
3. **异常不只涉及 yaw。** 150–160 秒窗口内 roll 约 −12.5° 到 +16.8°，pitch 约 −17.1° 到 +13.3°；高度随后降至地面附近。这与一次明显姿态扰动、随后降落相容，但单靠 LIO 无法确认真实机体运动。
4. **MAVROS 和 LIO 相似不等于两个独立传感器证实旋转。** Bridge 将 LIO orientation 原样发送给 MAVROS vision pose；若 PX4 融合该航向，两者可能相互关联。两条任务日志只能说明 PX4 输出也发生变化，不能区分机体旋转和估计器被外部姿态带偏。
5. **没有证据证明 EGO 下发了转圈指令。** 当前代码在 EGO 进入时锁定 yaw，之后保持；AlignDown 锁定 home yaw。Bag 没录实际 setpoint，不能完全排除运行版本差异或其他发布者。末次 AlignDown 切换到 home yaw 可能影响后半段，但其触发晚于异常开始。
6. **Bridge 的防跳变只管位置。** `lidar_to_px4_bridge.cpp:221` 原样转发 orientation，没有姿态质量判断。若异常源是 LIO，它可继续进入 PX4；本次 bridge 日志没有位置拒绝或重同步事件。LIO odometry covariance 全为 0，不能拿它判断估计置信度。
7. **航向不稳定事件不能错置到故障时刻。** EVENT 16642797 在本次飞行前初始化和最终停止传感器后出现，末段空中未见对应事件。PX4 源码该检查仅在未解锁时执行，因此既不能据此认定末段 EKF 发散，也不能靠空中没报它排除估计问题。

## 原因优先级及验证方法

首先检查本次 PX4 ULog 的 `battery_status` 电压/电流、`vehicle_angular_velocity`、`vehicle_attitude`、姿态/角速度 setpoint、电机输出和控制分配饱和状态。如果 yaw setpoint 接近固定、真实陀螺 z 角速度同时达到数百 °/s，且电压明显下陷/电机输出饱和，则支持动力不足或执行器问题；具体哪个电机/ESC异常还需对应输出及硬件检查。

如果四元数 yaw 大幅变化，而原始陀螺没有相应转动积分，则重点查 EKF yaw reset、外部视觉航向创新、磁场/磁航向创新和融合状态。若有真实转动但飞控 yaw setpoint 本身也大幅变化，应优先查控制命令或估计重置如何影响 setpoint。

下一次记录至少加入 `/mavros/battery`、`/mavros/state`、`/mavros/local_position/pose`、`/mavros/imu/data`、`/mavros/imu/data_raw`、`/mavros/vision_pose/pose`、`/mavros/setpoint_raw/local`、`/mavros/setpoint_position/local`、`/mavros/estimator_status`、`/livox/imu`。姿态估计回放还需要原始 LiDAR。实际话题是否存在需要按当时系统核对。

当前任务代码搜索未发现电池订阅/任务中断逻辑。日志显示低电量提示后仍继续 EGO、视觉对齐；后续应确认 PX4 电池保护动作，并让任务在严重低电量时停止任务并进入明确的降落流程。不能只调 yaw PID 或给 yaw 加滤波来处理这份记录。

## 输出

- `overview.png`：全程姿态和位置。
- `final_segment.png`：末段姿态/位置/变化率，标注低电量事件。
- `lio.csv`：所有四元数、姿态和位置样本。
- `metrics.json`：分段统计、峰值、时间线和 yaw 阈值时刻。
- `analyze.py`、`decode_events.py`：可复现分析脚本。
