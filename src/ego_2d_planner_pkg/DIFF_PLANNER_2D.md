# Diff-Planner 2D v1

本版以 DifferentialRobotics/Diff-Planner 的 `5f8551203426b371c22de55e5961d04cf7639c60`
为算法来源，替换原 EGO 三次 B 样条优化和执行核心。保留包名、节点名和外部话题，方便现有 FAST-LIO2 / PX4 程序接入。

## 算法和执行链

```text
/cloud_registered_filtered + /fastlio2/lio_odom
  -> 原二维点云投影、局部占用地图、45 cm 膨胀和历史障碍确认
  -> 二维 A* 绕行引导
  -> Diff MINCO：五次分段多项式 + 带状矩阵求解 + 伴随梯度
  -> Diff L-BFGS：联合优化 XY 途经点与各段时间
  -> 连续曲线碰撞 / 速度 / 加速度验收
  -> /ego_2d_planner/polynomial_2d
  -> traj_server_2d_node
  -> /position_cmd，固定 z，锁定 yaw，yaw_dot = 0
```

旧 `bspline_opt/` 文件仅供对照，CMake 不再编译它们，运行节点也不再调用它们。
地图、点云接口、历史占用缓存和二维 A* 是适配层；未引入上游的 ROS 1、三维地图、集群或飞控节点。

核心直接使用上游 MINCO / L-BFGS 头文件，出处、固定版本和许可证见
[third_party/Diff-Planner/NOTICE.md](third_party/Diff-Planner/NOTICE.md)。上游矩阵保留第三个空间列，
该列始终为零；优化变量只有 x、y 和分段时间，高度在控制消息中附加。

## 保留的外部接口

| 方向 | 话题 | 类型 / 含义 |
|---|---|---|
| 输入 | `/cloud_registered_filtered` | `sensor_msgs/PointCloud2`，FLOAT32 XYZ |
| 输入 | `/fastlio2/lio_odom` | `nav_msgs/Odometry`，统一世界坐标 |
| 输入 | `/simple_2d_planner/goal` | `geometry_msgs/PoseStamped`，使用目标 XY |
| 输出 | `/position_cmd` | 原 `quadrotor_msgs/PositionCommand`，50 Hz |
| 输出 | `/simple_2d_planner/local_goal` | 原 `geometry_msgs/PoseStamped` |
| 输出 | `/ego_2d_planner/raw_path` | 原 A* 可视化路径 |
| 输出 | `/ego_2d_planner/smooth_path` | MINCO 曲线的可视化采样 |
| 输出 | `/ego_2d_planner/selected_path` | 通过验收的 MINCO 曲线采样 |
| 输出 | `/ego_2d_planner/occupancy_grid` | 原二维占用地图 |
| 输出 | `/ego_2d_planner/cloud_2d` | 原二维投影点云 |
| 输出 | `/ego_2d_planner/local_goal_marker` | 原本地目标 Marker |
| 内部 | `/ego_2d_planner/emergency_stop` | 原 `EmergencyStop2D`，撤销与保持 |

可视化消息继续按订阅情况构造并复用缓冲。目标、轨迹、坐标数值允许随新算法改变。

**唯一必须切换的内部轨迹接口**：原 `/ego_2d_planner/bspline_2d` 改为
`/ego_2d_planner/polynomial_2d`，类型为新增的 `Polynomial2D`。
五次多项式无法普遍无损转换为旧三次 B 样条。旧 `Bspline2D` 定义保留供历史程序编译，
本版不再发布或订阅它。规划器与轨迹服务器须一起更新；飞控侧 `/position_cmd` 类型和字段保持一致。

`Polynomial2D` 包含 `header/start_time/traj_id/order/fixed_z/durations/coeff_x/coeff_y`。
`order=5`，第 i 段使用自己的局部时间 `t∈[0,durations[i]]`，
`coeff_x[6*i+k]` 与 `coeff_y[6*i+k]` 分别是 `t^k` 的系数，按低次到高次排列。
新服务器解析求位置、速度、加速度，并检查尺寸、有限数、段间 C2 连续和终点静止状态。

## yaw 和高度

启动后锁存首帧有效里程计四元数对应的 yaw，所有后续控制命令使用同一 yaw，`yaw_dot=0`。
更换目标、转弯、急停和恢复均不更新 yaw，不使用目标姿态或路径朝向来旋转飞机。
获得有效四元数前不发控制命令。改变初始航向需要以所需姿态重启节点。

轨迹只规划 XY，正常命令 z=1.0 m，z 速度 / 加速度为零；沿用 `use_msg_z/fixed_z`。
急停位置沿用旧版实际里程计 XYZ，以固定令牌位置持续保持。
这是命令层保持，没有新增动力学制动或起降控制。

## 避障、衔接和边界

- 占用地图仍为 10×10 m / 0.05 m，点云默认截取世界 z=0.1～2.0 m，离散膨胀半径 0.45 m。
  保留原持续观测确认与永久占用语义，不估计树中心。
- 碰撞约束采用占用格边界基点与指向绕行引导的脱障方向，在曲线积分点施加惩罚并传播至 MINCO 变量。
  本版不生成 ESDF。
- L-BFGS 联合优化空间和时间，含 jerk 积分、避障、引导贴合、速度 / 加速度惩罚和总时间代价。
  配置限速 0.35 m/s、限加速度 0.8 m/s²；验收不依赖软惩罚，也不放宽限值。
- 验收将五次曲线及其导数转换为 Bezier 控制多边形，递归检查包围盒占用和导数范数上界。
  包围盒覆盖整段连续曲线；未能确认安全就拒绝。边界贴格路径可能被保守拒绝。
- 初始 / 重规划 / 短距离 / 零距离目标均经过相同链路。至少两段 MINCO，避开上游单段伴随初始化缺失。
  时间映射附加 0.05 s 的单段下界，避免退化时间。
- 重规划从预定切换时刻的旧轨迹解析 P/V/A 出发。服务器保留旧轨迹直到切换时刻，
  并拒绝有 P/V/A 跳变的替代消息。正常规划起点假定启动时悬停；执行中的重规划继承上一轨迹。
- 执行时检查剩余曲线、飞机当前占用和跟踪偏差；新碰撞立即撤销旧轨迹。跟踪偏差阈值 0.50 m。
- 失败不执行原始 A* 或未验收曲线。原急停令牌的时间戳屏障、重复保持和新消息恢复机制继续生效。
- 用户最终目标不自动修改，目标被占据或超出局部地图则拒绝并保持 / 重试。
  本版不提供跨出局部窗口的全局导航，也没有移植多拓扑搜索或集群规划。
- ROS 时间回退会让服务器撤销当前轨迹；时间戳屏障仍保留，开始新时间纪元需要重启两个节点。

## 参数

新增 `diff/piece_length=0.50`、`diff/samples_per_piece=16`、`diff/max_evaluations=240`、
`diff/max_solve_ms=60.0`、`diff/weight_time=0.20`、`diff/weight_collision=2000.0`、
`diff/weight_feasibility=100.0`。每条轨迹最多 64 段。
求值次数和时间预算在重试间共享；60 ms 是协作式求解预算，单次求值和后验检查可能使总耗时超出预算，
不是硬实时保证。需要在 Orin 上实测后调整。

原 `optimization/max_iter/lambda_smooth/lambda_fitness/max_vel/max_acc/dist0/max_rebound_attempts/`
`retry_collision_scale/retry_smooth_scale` 保留并用于新核心。
`manager/bspline_sample_step` 名称为兼容保留，现在控制曲线可视化的时间采样步长。
`control_points_distance/knot_span/step_size/max_update/lambda_collision/lambda_feasibility/retry_dist0_scale`
以及旧的离散 `collision_check_step` 作为历史参数接受，但不控制 MINCO。
服务器限值 `max_vel/max_acc` 应与规划器一致；不一致时服务器可能拒绝轨迹。

## 编译与运行

ROS 2 Humble、C++17、Eigen3 和原 `quadrotor_msgs`。将源码包中的 `ego_2d_planner_pkg`
放入 ROS 2 工作区 src 后：

```bash
colcon build --packages-select ego_2d_planner_pkg --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch ego_2d_planner_pkg ego_2d_planner.launch.xml
```

启动名沿用旧接口，默认同时运行新规划器和新服务器。RViz 与三个配置文件均保留。
只启动 ROS 节点不会自动接管飞控或起飞。

```bash
ros2 topic pub --once /simple_2d_planner/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: lidar}, pose: {position: {x: 2.0, y: 1.0, z: 1.0}, orientation: {w: 1.0}}}"
```

## 本项目验证

WSL 编译目录单独使用 `/home/lin/ego_optimization_ros/diff`，未覆盖此前 v3 安装目录。
工作区中的复现工具：

```bash
bash tools/build_diff_ros.sh
bash tools/verify_diff_core.sh
bash tools/verify_diff_ros.sh
bash tools/verify_diff_replay.sh
```

核心单元测试随源码包提供：启用 `BUILD_TESTING` 后执行
`colcon test --packages-select ego_2d_planner_pkg`。
工具脚本的 WSL 构建目录、bag 路径和 `quadrotor_msgs` 依赖目录是当前主机路径，迁移时需调整。
实测结果见工作区 `analysis/diff/RESULTS.md`。bag 回放提供录制观测，并非飞机按新轨迹运动的闭环仿真。
