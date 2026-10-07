# door_offboard：识别穿门与 MAVROS 位置执行

独立 ROS2 Humble / C++17 功能包。不依赖 EGO、quadrotor_msgs、旧的 offboard_core_pkg 或 door_navigation 包，不包含 RViz、测试/演示可执行程序及降落程序。

本包参考已有 PresetpointTask → SetOffboardTask → ArmTask → TakeoffTask → HoverTask 的顺序。
默认自动起飞悬停到 MAVROS 本地坐标系 Z=0.6 m，稳定后开启识别；没有安全路径、等待下一道门、完成穿门时持续发送悬停位置。雷达驱动、FAST-LIO2、MAVROS、LIO 到 PX4 视觉定位桥由外部启动。

## 节点分工

| 节点 | 工作 |
|---|---|
| `flight_node`，ROS 名称 `door_flight` | 预发送、请求 Offboard、解锁、起飞、稳定悬停；把 LIO 目标转换到 MAVROS 本地坐标；沿直线生成限速限加速度的位置参考；到点反馈、无路径保持和手动接管 |
| `perception_node`，ROS 名称 `door_perception` | 等待飞行节点启用；读取点云/LIO；水平切片、门洞识别、障碍膨胀、当前直线净空检查（未知区不拦截）；发布目标和几何路径 |

只有 flight_node 向 `/mavros/setpoint_position/local` 发布，频率默认 20 Hz。不得同时运行原 offboard 控制节点、EGO 发送器或其他同话题控制源。
本包不调用降落服务、不请求 AUTO.LAND、不发送解锁=false。

## 编译与启动

将整个 `door_offboard` 放入现有工作空间 `src`。需要 ROS2 Humble 的 mavros_msgs、rosidl_default_generators、Eigen、OpenCV 与 nlohmann_json 开发依赖。

```bash
cd ~/mavros_ws
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
colcon build --packages-select door_offboard --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

先外部启动 MID360、FAST-LIO2、MAVROS 及原有定位桥，确认本地位姿持续有效。以下启动命令会自动进入 Offboard、解锁并起飞，不是单纯的识别预览：

```bash
ros2 launch door_offboard door_offboard.launch.xml
```

修改高度及其他参数：直接编辑 `~/mavros_ws/src/door_offboard/config/door_offboard.yaml`，重新编译并重启。按上述 symlink-install 方式安装时，先确认安装目录的配置链接指向该文件。

本次完整包沿用最近实测配置：高度0.6m、速度0.2m/s、悬停持续1s、高度容差0.10m、停止速度0.10m/s、到点容差0.10m、输入超时3s。识别节点的到点容差/停止速度与控制端一致；识别高度容差仍为0.06m。

默认悬停高度参数为 0.6 m，直接作为 MAVROS 本地 ENU 坐标系的目标 Z，不叠加启动时的 Z。修改参数后重启生效。
例如启动 Z=0.1 m，配置 hover_height_m=0.6 时，起飞、穿门和等待的目标 Z 均为 0.6 m。只有本地坐标原点位于地面时，这才对应离地 0.6 m；本包不测量地面高度。
`auto_start: false` 仅用于启动节点检查接口，不执行起飞；需要修改配置并重启才能启动任务。

## 坐标与高度

- `/fastlio2/world_cloud`：sensor_msgs/PointCloud2，世界系默认 `lidar`。
- `/fastlio2/lio_odom`：nav_msgs/Odometry，header.frame_id=`lidar`，child_frame_id=`body`。
- `/mavros/local_position/pose`：geometry_msgs/PoseStamped，默认 frame_id=`map`；使用 MAVROS ROS 端的本地 ENU 坐标。
- `/door/goal` 和 `/door/path`：LIO 世界系。
- `/door/target_pose` 和 `/mavros/setpoint_position/local`：MAVROS 本地坐标；XYZ 以米为单位，Z 为固定起飞目标。

起飞后稳定悬停，使用时间配对的 LIO 与 MAVROS 位姿锁定平面旋转和平移：`p_local = R * p_lio + t`。
两套姿态必须描述相同机体轴、重力对齐且单位一致；本功能不估计安装轴差、雷达标定或完整三维外参。
`body_offset` 是 LIO 原点到与 MAVROS 一致的机体参考点，两节点必须一致；`lidar_offset` 用于识别节点的射线原点。
外部 LIO→PX4 定位桥仍然必需，本包的目标点映射不替代它。不能靠改 frame 名解决坐标轴或参考点不一致。

固定 LIO 切片高度由悬停配对位姿与固定 MAVROS Z 计算，通过启用消息传给识别节点，不需要填写 flight_z。
飞行过程中不自动重新校准坐标关系；两套位姿映射误差超过 0.15 m 或航向差变化超过 0.2 rad 时进入 FAULT_HOLD。
不使用顶网作为高度参考。0.3 m 低空时必须按实物核对 body_below、body_above、vertical_margin 与参考点位置，默认尺寸只是沿用原配置。

## 起飞与穿门流程

1. WAIT_INPUT：等待新鲜且有效的 MAVROS 状态、本地位姿和 LIO 位姿。程序启动时飞机已解锁则退出自动流程，避免重启造成二次起飞。
2. PRESTREAM：在当前位置持续发送位置点，默认 2 秒。
3. OFFBOARD → ARM：请求模式和解锁，以实际状态消息确认为准，默认超时 10 秒。
4. TAKEOFF → STABILIZE：固定起飞 XY/yaw，目标 Z 设为 hover_height_m（默认 0.6 m）。高度误差≤10 cm、水平误差≤10 cm且速度≤10 cm/s，持续 1 秒后开启识别；识别节点另要求与固定 LIO 高度误差≤6 cm。
5. NAVIGATE：无路径时保持悬停；识别稳定后生成门前 0.5 m 和门后 0.5 m 两个端点。
6. APPROACH：只执行当前位置到门前点这一条已经通过障碍和地图边界检查的直线。
7. 收到匹配会话/目标编号的到达反馈，并由 LIO 确认到点后，切到 CROSS；检查通过才执行到门后点。
8. 到达门后点计一次，再识别下一道门。默认最多两次；`max_doors: 1` 可指定只通过一道。
9. 完成后 COMPLETE_HOLD，继续发布悬停位置，不降落。默认两道但只检测到一道时，继续悬停等待，不会自行搜索移动或降落。

阶段目标固定为门中心前0.5m和门中心后0.5m，两目标相距1m；到达前一点才切换下一点。仍通过MAVROS位置接口发送中间参考点，参考推进速度上限 `max_speed=0.20 m/s`，轨迹加速度参数 `max_acceleration=0.30 m/s²`，不修改PX4参数。

采用可暂停的轨迹时间：下一参考点距实际XY超过 `tracking_error_m=0.12 m` 时，持续发送当前参考点并等待；追上后从原进度继续，不清空轨迹、不把参考点退回实际位置，也不重新从零加速。日志增加 `tracking_wait` / `tracking_resumed`，状态原因为 `Waiting for position tracking`。限速针对发送的参考点，实际机体速度受PX4控制响应影响。

控制端与识别端的XY到达容差均为0.10m，保留停止速度、固定高度和0.3秒到达停留条件。
门前/门后两点的标称间距是 1 m，因实际到点容差为 10 cm，第二段从实际位置到固定门后点的剩余长度可能略有不同。
实际位置达到终点容差且速度低于阈值，持续 0.3 秒后反馈到达；不会仅凭轨迹时间结束换段。

## 没有路径与中断行为

`/door/path` 是完整几何参考，即使当前段不可执行也可存在；只有 `/door/goal.valid=true` 且消息、位姿都新鲜时才执行。
直线被已检测障碍物的膨胀区域阻塞或超出地图边界时，不绕行、不缩短，保持当前 XY 与固定 Z。未知区域不再参与起点、终点、线段或通用阻塞地图的通行判断。

已按用户要求取消全部未知区域拦截，包括门板后方未观测区域。障碍物仍按 `radius + margin`（当前0.25m）膨胀；没有障碍点不代表物理空间一定没有障碍，本版本允许规划通过未观测区域。

原有墙体线段重建仍用于补齐已检测障碍表面；未知层和观测统计仅保留作诊断，不与执行阻塞地图合并。日志 `scene.unknown_blocks_path=false` 表示本版本已取消未知区拦截。`corridor_model_active` 和 `inferred_free_cells` 仍为诊断数据，不再决定是否允许通过未知区域。

本次只修改未知区通行策略；此前发现的消息时间戳略微超前可能触发 FAULT_HOLD 的问题尚未修改。

规划消息超时默认 0.6 秒，点云/LIO 过期会撤销目标有效性；输入恢复且停稳后可以从当前位置继续同一目标。
悬停点在进入保持时锁定，不是每一帧跟随当前位置漂移。短暂漏检不改变已经锁定的门。

传感器失效、连接丢失、起飞/到达超时或坐标关系异常进入 FAULT_HOLD，禁止自动恢复任务，仍尝试发布保留的悬停设定点。
悬停效果依赖 PX4 定位和链路；本包不覆盖 PX4 自身的 Offboard 丢失保护。

手动切出 Offboard 或飞机解除解锁后进入 MANUAL：停止自动目标执行与 MAVROS 设定点发布，不再请求 Offboard/解锁。
由你使用遥控器/已有方式手动降落；本包不包含降落任务。程序不会通过反复请求模式抢回控制权。

## 输出和内部话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/mavros/setpoint_position/local` | geometry_msgs/PoseStamped | 实际发给 MAVROS 的连续位置参考，默认 20 Hz |
| `/door/target_pose` | geometry_msgs/PoseStamped | 当前有效阶段的最终目标 XYZ/yaw，MAVROS 本地坐标；不是逐时刻中间参考 |
| `/door/goal` | door_offboard/DoorGoal | 原子消息：会话ID、目标ID、有效性、完成状态、阶段、原因与 LIO 目标 |
| `/door/enable` | door_offboard/PlannerEnable | 飞行节点的识别启用心跳，以及固定 LIO 高度 |
| `/door/reached` | door_offboard/GoalReached | 已到达的会话/目标编号；识别端验证后切换阶段 |
| `/door/path` | nav_msgs/Path | LIO 世界系完整几何路径，保持同一个 Z，无升降规划 |
| `/door/detection` | std_msgs/String | JSON 门中心、方向、宽度、稳定次数，确认阶段发布 |
| `/door/status` | std_msgs/String | JSON 识别/规划阶段、有效性、原因、已过门次数 |
| `/door/flight_status` | std_msgs/String | JSON 飞行阶段、固定高度、目标编号、保持状态及实际发布参考 |
| `/door/debug_image` | sensor_msgs/Image | OpenCV 识别用二维占据切片 |

```bash
ros2 topic echo /door/flight_status
ros2 topic echo /door/status
ros2 topic echo /door/target_pose
ros2 topic hz /mavros/setpoint_position/local
```

两节点启动即自动记录 JSONL 日志，默认保存为 `~/.ros/door_offboard/door_flight_<时间戳>.jsonl` 和 `~/.ros/door_offboard/door_perception_<时间戳>.jsonl`。每次启动创建新文件；终端打印完整文件路径。若设置了 ROS_HOME，默认目录为 `$ROS_HOME/door_offboard/`。

飞行日志按控制频率（默认20 Hz）记录实际 MAVROS/LIO 位置、速度、发送参考、目标误差、输入时间延迟、规划有效性与飞行模式；另记录 Offboard/解锁请求及结果、状态切换、坐标转换、目标执行、到达和悬停原因。
识别日志按规划频率（5 Hz）记录切片高度、障碍/已观测栅格数量、门中心及稳定次数、锁定门、完整几何路径、当前目标、阻塞原因和到达反馈；每次接收并处理点云时记录点数、时间戳配对误差，丢弃不同步点云时记录原因。
日志每行包含 ROS 时间 `stamp`、系统时间 `wall_stamp`、进程运行时间 `elapsed_s`、节点名、事件名和数据，方便将两个文件按 ROS 时间、session_id、goal_id 对齐。关键事件立即刷盘，周期记录最多每1秒刷盘；正常退出写入 shutdown。进程被强制杀死或断电时不保证写入最后1秒的数据。

两个节点的 `enable_logging` 均默认为 true。自定义位置时把配置中两处 `log_directory` 改为同一绝对路径，例如 `/home/jetson/door_logs`。运行中可查看：

```bash
ls -lt ~/.ros/door_offboard/
tail -f "$(ls -t ~/.ros/door_offboard/door_flight_*.jsonl | head -n 1)"
```

测试结束、停止程序后，打包最近启动的两份日志给我分析：

```bash
cd ~/.ros/door_offboard
flight_log=$(ls -t door_flight_*.jsonl | head -n 1)
perception_log=$(ls -t door_perception_*.jsonl | head -n 1)
tar -czf "$HOME/door_logs_$(date +%Y%m%d_%H%M%S).tar.gz" "$flight_log" "$perception_log"
```

日志记录数值和诊断，不保存完整点云及图像；重现现场点云仍需 rosbag。
不包含测试代码或测试节点。交付验证结果另见随包的验证说明；模拟 MAVROS 接口测试不等同于 PX4 SITL 或实机验证。
