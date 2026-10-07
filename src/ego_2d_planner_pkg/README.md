# ego_2d_planner_pkg — Diff-Planner 2D v1

当前版本已替换为 **Diff-Planner 五次 MINCO + L-BFGS 空间 / 时间联合优化**。
正常高度固定，yaw 锁存首帧有效里程计，所有命令 yaw_dot=0。

包名、节点名、点云 / 目标 / 路径 / `/position_cmd` 接口沿用旧版。
内部轨迹切换为 `Polynomial2D`，规划器与轨迹服务器需要一起更新。
旧三次 B 样条代码仅作参考，不参与编译或运行。

完整算法、消息说明、参数、限制和测试方式见 [DIFF_PLANNER_2D.md](DIFF_PLANNER_2D.md)。

```bash
colcon build --packages-select ego_2d_planner_pkg --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch ego_2d_planner_pkg ego_2d_planner.launch.xml
```

默认保留 10×10 m / 5 cm 地图、45 cm 膨胀、0.35 m/s 限速、
0.8 m/s² 限加速度、10 Hz FSM 和 50 Hz PositionCommand。
通过连续曲线碰撞和动力学验收后才发出轨迹，急停撤销与时间戳屏障保留。

算法来源与许可证见 [上游出处](third_party/Diff-Planner/NOTICE.md)。
历史优化记录：[v3](OPTIMIZATION_V3.md)、[v2](OPTIMIZATION_V2.md)、[v1](OPTIMIZATION.md)。