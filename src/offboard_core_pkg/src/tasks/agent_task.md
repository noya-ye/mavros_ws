# Offboard Core Task 开发规范

本文是 `src/offboard_core_pkg` 中编写和维护 task 的事实指南。新增或修改 task 时，先阅读本文及相关头文件；实现完成后必须同步更新本文的“现有 task 接口登记”。坐标统一使用 MAVROS 暴露的 ROS ENU（x 东、y 北、z 上）坐标，MAVROS 在桥接边界处理 ENU/NED 转换。

## 1. Task 的职责边界

- task 是一个可被 `Scheduler` 驱动的有限状态行为，不创建 ROS timer、不直接 spin，也不直接创建 publisher/client。
- 任务状态通过 `Context` 读取或写入：飞行器连接/解锁/模式、位置/速度/yaw、home、位置和 yaw setpoint、fault 等。
- 所有 MAVROS 服务请求通过 `MavrosIface`：`requestMode`、`requestArm`、`requestLand`。这些调用是异步的，回调只记录结果；不要阻塞等待 future。
- 位置控制通过写入 `ctx.position_setpoint_enu`、`ctx.yaw_setpoint_enu`，并保持 `ctx.publish_position_setpoint = true`；节点定时器每周期调用 `iface.publishSetpoint()`。
- 原始 setpoint 模式通过 `ctx.setpoint_mode` 选择：`POSITION`、`POSITION_VELOCITY`、`POSITION_VELOCITY_ACCELERATION` 或 `VELOCITY_ONLY`。速度/加速度分别写入对应 ENU 字段，接口通过 `/mavros/setpoint_raw/local` 发布 `PositionTarget` 并转换为 `FRAME_LOCAL_NED`。旧代码设置 `use_position_velocity_acceleration = true` 时仍强制使用全量位置+速度+加速度模式。
- task 不应修改不属于自己的全局流程状态；失败时设置可诊断的 `ctx.fault`，成功/失败由 `ITask::Status` 返回。

## 2. ITask 生命周期与实现要求

```cpp
class MyTask final : public ITask {
public:
  explicit MyTask(/* validated parameters */);
  std::string name() const override;
  void onEnter(Context &, MavrosIface &) override;
  Status tick(Context &, MavrosIface &, double dt_s) override;
  void onExit(Context &, MavrosIface &) override;       // 需要清理时实现
  bool canPause(const Context &) const override;        // 默认 true
  void onPause(Context &, MavrosIface &) override;
  void onResume(Context &, MavrosIface &) override;
  void onCancel(Context &, MavrosIface &) override;
};
```

- `name()` 必须稳定且唯一；它用于日志、`Scheduler::interrupt` 和 `jumpTo`。现有普通流程名使用小写，`SnakeGridTask` 当前使用 `SNAKE_GRID`，不要无意改变已有名称。
- `onEnter` 每次进入或重新开始时重置 elapsed、请求 pending、阶段和目标初始化标志，并通常清空 `ctx.fault`。
- `tick` 每周期执行一次。对 `dt_s` 使用 `std::max(0.0, dt_s)`；需要更强鲁棒性时同时处理非有限值。只返回 `RUNNING`、`SUCCESS` 或 `FAILURE`，不要在 tick 内重复调用生命周期函数。
- 超时、非法输入、无可行路径等不可恢复条件返回 `FAILURE` 并设置 `ctx.fault`；等待连接、有效位置或异步响应时返回 `RUNNING`。
- 析构必须安全；task 不能让异步回调访问已经销毁的外部对象。当前服务 task 使用 `[this]` 回调，因此只允许在对象仍由 scheduler 持有期间发起请求。

## 3. Scheduler 使用规则

- `scheduler.add(std::make_unique<MyTask>(...))` 加入顺序流程；`addAux` 加入不计入顺序进度的辅助 task。加入后调用 `reset()`。
- 每个周期由节点调用 `scheduler.tick(ctx, iface, dt_s)`；Scheduler 自动调用 `onEnter`、`tick`、`onExit`。
- `SUCCESS` 进入下一个顺序 task；`FAILURE` 使 scheduler 同时 `failed()` 和 `done()`。
- `interrupt(name, ctx, iface, RESUME_CURRENT)` 会先调用当前 task 的 `canPause/onPause`，目标成功结束后调用 `onResume`；`DROP_CURRENT` 会调用 `onCancel` 并丢弃当前任务。需要保持飞行安全时覆盖 `canPause` 并拒绝不安全暂停。
- `jumpTo` 会取消当前和已暂停 task，不恢复它们。不要在 task 内自行跳转 scheduler。
- `current_name/current_index/total_count` 可用于外部状态展示；不要依赖内部 `idx_`。

## 4. Context 与 setpoint 约定

`Context` 的输入状态由 `MavrosIface` 订阅回调更新：`connected`、`armed`、`mode`、`position_valid`、`position_enu`、`velocity_enu`、`yaw_enu`。第一次收到有效位置时自动初始化 `home_enu/home_yaw_enu` 及初始 setpoint。

- 读取位置前检查 `ctx.position_valid`，必要时再检查 `ctx.finitePosition()`。
- 目标点应在 `onEnter` 或首次有效 tick 时捕获，避免每周期把目标重置到当前位置。
- `publish_position_setpoint=false` 会让 `publishSetpoint()` 完全不发布；普通位置 task 应明确保持为 true。
- yaw 使用弧度；高度和距离使用米；持续时间和 `dt_s` 使用秒。
- 不直接写 `ctx.connected/armed/mode`，这些字段反映 MAVROS 反馈。服务是否接受由回调或后续状态反馈确认。

## 5. 异步命令、重试与错误处理

命令型 task 推荐维护 `elapsed_s_`、`retry_elapsed_s_`、`request_pending_`，在未 pending 且达到重试间隔时调用接口。`requestMode/requestArm/requestLand` 返回 `false` 表示 client 尚未 ready，应继续等待并重试；返回 `true` 只表示请求已发出，不表示最终成功。超时必须覆盖 client 不可用和服务端拒绝。

构造函数应将外部参数归一化（例如 `std::max(0.0, timeout_s)`），对网格、步长、容差等参数做完整合法性检查。不要吞掉错误；`ctx.fault` 应包含 task 名称相关的简短原因。

## 6. 新增 task 的落地步骤

1. 在 `include/offboard_core_pkg/tasks/<name>_task.hpp` 声明 `final` 类、构造参数、ITask 生命周期和必要的查询接口。
2. 在 `src/tasks/<name>_task.cpp` 实现状态机；只依赖 `context.hpp`、`mavros_iface.hpp` 和必要的本地 helper。
3. 将源文件加入 `CMakeLists.txt`，将头文件加入 `include/offboard_core_pkg/tasks.hpp`；在节点或调用方用 `std::make_unique` 注册并 `scheduler.reset()`。
4. 设计暂停、恢复、取消语义；运动 task 的暂停至少应把 setpoint 收回当前位置，恢复时重新校准目标或索引。
5. 编译并运行可用测试；至少覆盖参数边界、未连接/无位置、成功、超时/失败，以及 scheduler 的顺序或中断行为。
6. 在本文登记新 task 的构造函数、`name()`、生命周期、副作用、完成条件、失败条件和公开查询接口。任何接口或功能变化都必须修改对应登记，而不是只改实现。

## 7. 现有 task 接口登记

### `ArmTask`

- 构造：`ArmTask(double timeout_s = 10.0, double retry_interval_s = 1.0)`。
- `name()`：`"arm"`。
- 行为：进入时清理计时、请求状态和 fault；已 `ctx.armed` 则成功，否则通过 `requestArm(true, callback)` 周期重试，超过 timeout 失败（`arming timed out`）。
- 公共生命周期：`onEnter`、`tick`；使用 ITask 默认 pause/resume/cancel。

### `SetOffboardTask`

- 构造：`SetOffboardTask(double timeout_s = 10.0, double retry_interval_s = 1.0)`。
- `name()`：`"set_offboard"`。
- 行为：已处于 `OFFBOARD` 则成功，否则通过 `requestMode("OFFBOARD", callback)` 重试；超时失败（`OFFBOARD mode request timed out`）。
- 公共生命周期：`onEnter`、`tick`；默认可暂停。

### `TakeoffTask`

- 构造：`TakeoffTask(double height_m, double tolerance_m, double timeout_s = 30.0)`。
- `name()`：`"takeoff"`。
- 行为：等待连接、有效位置和解锁；首次满足条件时在已有 XY/yaw setpoint 上增加 `height_m`，高度误差不超过 tolerance 成功，超时失败（`takeoff timed out`）。依赖前序 `PresetpointTask` 提供安全参考点。
- 公共生命周期：`onEnter`、`tick`；默认可暂停。

### `HoverTask`

- 构造：`explicit HoverTask(double duration_s)`。
- `name()`：`"hover"`。
- 行为：累计非负 dt，持续 duration 后成功；不主动修改 setpoint。
- 公共生命周期：`onEnter`、`tick`；默认可暂停。

### `PresetpointTask`

- 构造：`explicit PresetpointTask(double duration_s)`。
- `name()`：`"presetpoint"`。
- 行为：等待连接和有效位置，首次有效时捕获当前位置及 yaw 为 setpoint，保持 duration 后成功；用于 OFFBOARD warm-up 前的安全参考点。
- 公共生命周期：`onEnter`、`tick`；默认可暂停。

### `LandTask`

- 构造：`LandTask(double timeout_s = 15.0, double retry_interval_s = 1.0)`。
- `name()`：`"land"`。
- 行为：未解锁即成功；进入时捕获有效本地位置的 XY（若进入时尚无定位，则首次有效 tick 捕获），随后始终保持该 XY。任务将高度 setpoint 设为 `ctx.home_enu.z + 0.1 m`，等待实际高度进入约 0.05 m 容差后，再通过 `requestLand(callback)` 周期重试开启 LAND；超过 timeout 失败（`landing command timed out`）。等待连接、有效位置或 home 初始化期间保持 `RUNNING`。
- 公共生命周期：`onEnter`、`tick`；默认可暂停。

### `SnakeGridTask`

- 构造：`SnakeGridTask(rclcpp::Logger logger, const Config &cfg)`。
- `name()`：`"SNAKE_GRID"`。
- `Config`：`FirstAxis {X_FIRST,Y_FIRST}`、`StopMode {EVERY_CELL,LINE_END_ONLY}`、`x_cells/y_cells`、`cell_size`、`x_sign/y_sign`、`include_start_cell`、`hover_s`、`max_step_m`、`arrive_xy_m/arrive_z_m`、`obstacle_cells`。
- 行为：`onEnter` 检查位置和配置，以当前 XY/高度/yaw 构造蛇形网格；支持障碍物绕行、每格或每行末悬停，逐步发布位置 setpoint。非法配置、无效位置或无路径失败；所有航点完成成功。
- 公共生命周期：`onEnter`、`tick`、`onExit`、`onPause`、`onResume`；暂停时收回当前位置，恢复时在近期航点中重新定位。
- 查询接口：`planId()`、`planReady()`、`routeCells()`、`currentIndex()`、`totalWaypoints()`、`currentCell()`、`finished()`、`failed()`。

### `AlignDownTask`

- 构造：`AlignDownTask(double pixels_per_meter, int stable_frames, double arrive_distance_m, double max_step_m)`；比例单位是像素/米，帧数、距离阈值与最大单帧步长必须为正数；浮点参数还需为有限值。
- `name()`：`"align_down"`。
- 行为：使用 `Context` 中下视圆心偏差，圆心超过 0.5 秒未更新时回退到轮廓中心。图像上/左分别视作机体前/左，按当前 yaw 转到 ENU，保持偏差方向，将水平纠偏距离限制为 `max_step_m` 后加到当前 XY；Z 固定为进入任务时的高度，yaw setpoint 锁定为 `home_yaw_enu`。超出阈值继续修正；第一次进入阈值时检查 YOLO：接收时间不超过 0.5 秒、序号非零且存在置信度严格大于 0.40 的检测，打印 `FIND TARGET`；否则打印 `NO TARGET`、保持位置并返回 `SUCCESS`，跳过稳定帧等待并继续后续流程。circle 或 contour 当前检测偏差进入到达阈值时均可判定到达。确认只执行一次，`onEnter` 重置，暂停恢复或再次进入阈值不重复确认；确认通过后连续收到 `stable_frames` 个阈值内的新检测帧后成功。无新检测时不累计帧数，检测超时后保持当前位置并清零稳定计数。
- 失败条件：构造参数无效，设置 `ctx.fault` 后返回 `FAILURE`。无连接/定位或目标暂时丢失则等待，不会误判成功。
- 生命周期：`onEnter`、`tick`、`onPause`、`onResume`；暂停时保持当前位置，恢复时等待新检测帧重新计数。
- 查询接口：`reached_arrival_tolerance()` 表示检测偏差曾进入阈值；`target_confirmed()` 表示该次校准通过 YOLO 确认目标，而不是 `NO TARGET` 跳过。
- 节点接入：`offboard_core_node` 通过 `align_down.enabled`（默认 true）在悬停与降落之间插入本任务；组合任务节点也使用相同检测输入。circle 优先，circle 数据过期时，与最近 circle 点相距小于 40 像素的 contour 按 circle 等效进行到达判定。相机需要使图像上/左与机体前/左一致，安装方向不同时需先调整坐标映射。

### `RedCrossAlignTask`

- 构造：`RedCrossAlignTask(double pixels_per_meter, int stable_frames, double arrive_distance_m, double max_step_m)`；参数边界与 `AlignDownTask` 相同。
- `name()`：`"red_cross_align"`。
- 行为：只读取 `Context` 中最新的 red-cross 图像中心偏差，不做 circle/contour 回退或 YOLO 筛选。图像上/左分别映射为机体前/左，以当前 yaw 转为 ENU；单帧限步长规则与 `AlignDownTask` 相同。Z 固定为进入任务时高度，yaw 锁定为 home yaw。检测超过 0.5 秒未更新时悬停并清零稳定帧计数；误差进入阈值后，连续收到 `stable_frames` 个新帧才成功。
- 失败条件：构造参数无效时设置 `ctx.fault` 并返回 `FAILURE`；无连接/定位、无效 yaw 或暂时丢失目标时等待并保持安全位置。
- 生命周期：`onEnter`、`tick`、`onPause`、`onResume`；暂停时保持当前位置，恢复后从新的视觉帧重新计数。无公开查询接口。
- 节点接入：`offboard_core_node` 的 `red_cross_align.enabled`（默认 false）启用后订阅 `red_cross_align.topic`（默认 `/target/red_cross_center`），沿用其现有 AlignDown 校准参数。`AlignDropSnakeEgoTask` 则通过 `red_cross_align.*` 配置独立校准参数。`camera_center_node` 发布同名话题，消息使用与现有中心点相同的上/左为正偏差约定。

### `DownDropTask`

- 构造：`DownDropTask(rclcpp::Logger, double land_height, obj_id target, std::string serial_device, unsigned int baud_rate = 115200)`。
- `name()`：`"down_drop"`。
- 行为：先移动到配置高度，再移动目标 XY 偏移；位置满足 `GotoTask` 容差后返回 `SUCCESS`。当前串口发送段仍为注释代码，因此任务成功不代表已向投放器发出命令。
- 失败条件：投放配置或进入时位置无效，或内部位置任务失败时设置 `ctx.fault` 并返回 `FAILURE`。
- 生命周期：`onEnter`、`tick`、`onExit`；退出时关闭串口。无公开查询接口。

### `EgoVelFollowTask`

- 构造：`EgoVelFollowTask(rclcpp::Logger, const EgoVelPlanner::Config &)`。
- `name()`：`"EGO_VEL_FOLLOW"`。
- 行为：消费 `Context` 中的 EGO PositionCommand/里程计，持续生成当前 MAVROS 位置、速度和加速度 setpoint；输入无效或超时则位置悬停。

### `EgoGotoTask`

- 构造：`EgoGotoTask(rclcpp::Logger, rclcpp::Clock::SharedPtr, PoseStamped publisher, const Config &)`。
- `name()`：由 `Config::task_name` 指定，默认 `"EGO_GOTO"`。
- 行为：向配置目标发布 `PoseStamped`，跟踪 EGO 输出并在位置、速度同时稳定达到容差后返回 `SUCCESS`；规划器异常时保持当前位置。

### `CorridorDoorTask`

- 构造：`CorridorDoorTask(rclcpp::Logger, rclcpp::Clock::SharedPtr, PoseStamped publisher, const Config &)`；配置包含 `door_count`、占据阈值、门洞最小/最大宽度、前向搜索范围、地图超时、goto 容差、穿门距离、阶段超时及 `EgoGotoTask::Config`。
- `name()`：`"CORRIDOR_DOOR"`。
- 行为：逐列扫描飞机 ENU x 正方向的新鲜 `/ego_2d_planner/occupancy_grid`。只接受在同一 x 列中由 occupied 单元夹住、宽度处于最小值和最大值之间的连续 free 区域，unknown 不视为 free；从每列候选中选择离飞机 y 最近的开口中点，交给 `EgoGotoTask`。到达后用 `GotoTask` 向 ENU +x 飞行 `crossing_distance_m`（默认 0.1 m），完成后增加穿门计数并继续扫描，达到 `door_count` 后成功。
- 失败条件：单阶段超过 `stage_timeout_s`、EGO 或 goto 子任务失败时设置 `ctx.fault` 并返回 `FAILURE`；等待连接、定位、地图新鲜或尚未找到合格门洞时保持当前位置。
- 生命周期：`onEnter`、`tick`、`onExit`；运动期间拒绝 scheduler pause，退出时让位置 setpoint 回到当前位置。
- 查询接口：`doorsCrossed()` 返回本轮已完成门数。
- 节点接入：`corridor_door_node` 订阅 EGO 的 PositionCommand、Odometry 和 OccupancyGrid，按预设点、OFFBOARD、解锁、起飞、穿门、降落顺序调度。`corridor.door_count` 配置门数；其余门洞扫描参数见 `config/corridor_door.yaml`。

### `SnakeEgoAvoidTask`

- 构造：`SnakeEgoAvoidTask(rclcpp::Logger, rclcpp::Clock::SharedPtr, PoseStamped publisher, const Config &)`。
- `name()`：`"SNAKE_EGO_AVOID"`。
- `Config`：包含 `SnakeGridTask::Config`、`EgoGotoTask::Config`、`trigger_distance_m`、`occupancy_timeout_s`、`avoidance_timeout_s`、`occupied_threshold` 和内部降落参数。
- 行为：执行 `SnakeGridTask` 蛇形覆盖；读取 EGO 膨胀后的 `OccupancyGrid`，当当前航段进入可配置触发距离时，将当前或后续第一个未占据航点交给 `EgoGotoTask`。被占据航点跳过，EGO 成功后从该航点之后继续蛇形遍历。
- 失败处理：EGO 超时、规划失败、地图无安全航点或蛇形任务失败时进入内部 `LandTask`；降落完成后以 `FAILURE` 结束并保留故障原因。
- 公共生命周期：`onEnter`、`tick`、`onExit`；内部状态为蛇形、EGO 避障、降落和失败。

### `AlignDropSnakeEgoTask`

- 构造：`AlignDropSnakeEgoTask(rclcpp::Logger, rclcpp::Clock::SharedPtr, PoseStamped publisher, const Config &)`；配置包含蛇形、EGO、避障和 AlignDown 参数，`align_timeout_s` 默认 10 秒。
- `name()`：`"ALIGN_DROP_SNAKE_EGO"`。
- 行为：蛇形遍历与 EGO 避障时检查新鲜的 RedCross、下视 circle 和 contour 数据；同时有效时 RedCross 优先，开始后两种校准不会互相打断。RedCross 使用独立参数；RedCross 成功或 AlignDown 成功且 YOLO 确认目标时执行 `DownDropTask`。每次 DownDrop 依次使用 `id=0/1/2` 的目标偏移；第三次投放成功后返回纠偏开始位置并结束组合任务，不再恢复蛇形/EGO。前两次投放成功后返回并恢复原阶段。Align 检查优先于障碍触发和 EGO tick。
- 防重复：触发后锁存至本次 AlignDown 成功或失败。AlignDown 首次进入到达阈值时，优先记录新鲜圆心对应的 ENU 目标位置，其次记录新鲜轮廓投影位置，最后使用此时位于到达阈值内的飞机 ENU 位置估算。每次成功对齐的位置保留在本次任务运行的列表中；新的轮廓位置按像素比例和飞机 yaw 投影到 ENU，距任一已完成位置不超过 `align_retrigger_radius_m`（默认 0.55 m）时跳过重复对齐。`NO TARGET` 不会清除已记录位置。
- 失败处理：AlignDown 或 RedCross 超时后返回纠偏前位置并恢复原阶段；校准任务失败或 DownDrop 失败时返回 `FAILURE`。蛇形/EGO 避障故障沿用内部降落处理。
- 节点接入：`snake_ego_avoid_node` 订阅 `/target/contour_center` 与 `/target/circle_center`；`align_drop_snake_ego_node` 还可订阅 `/target/red_cross_center`。话题、独立标定参数、超时及 DownDrop 参数在各自 YAML 配置中设置。

## 8. 维护检查清单

当新增 task，或修改构造参数、`name()`、生命周期覆盖、setpoint/服务副作用、成功/失败条件、公开查询接口时：同步修改本文登记；检查 `tasks.hpp`、`CMakeLists.txt`、节点注册和测试；在变更说明中指出是否影响 scheduler 中断/恢复语义。
