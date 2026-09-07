# YZQ rectangular obstacle field

The Gazebo world uses the ENU metre frame with `O=(0,0)` at the circular
marker centre: `x` increases right/east and `y` increases up/north.

Field: `x in [-5.0, 5.0]`, `y in [-0.5, 8.0]` (10.0 m by 8.5 m).
Marker: centre `O=(0.0,0.0)`, radius `r=0.5 m`; it is tangent to the lower
edge at `T=(0.0,-0.5)`. Thus the centre-to-left distance is 5.0 m and
centre-to-lower-edge distance is 0.5 m. The positive axes are +x right and
+y up.

Obstacle centres (each footprint is 0.8 m x 0.8 m, height 1.8 m):

| ID | centre (x, y) m |
|---|---|
| 01 | (-3.8, 1.0) |
| 02 | (0.0, 1.8) |
| 03 | (3.6, 1.0) |
| 04 | (-2.5, 3.8) |
| 05 | (1.0, 4.2) |
| 06 | (3.8, 5.8) |
| 07 | (-3.8, 6.8) |
| 08 | (0.0, 6.8) |

The nearest obstacle footprint edges are more than 0.8 m apart; all footprints
remain inside the field boundaries. Use `PX4_GZ_WORLD=yzq_rectangular_obstacle_field`
with `PX4_GZ_WORLDS` pointed at this directory.
已创建仿真场地文件：

  - simulation/worlds/yzq_rectangular_obstacle_field.sdf
  - 二维平面图 SVG (simulation/worlds/yzq_rectangular_obstacle_field.svg)
  - 坐标与障碍物说明 (simulation/worlds/README.md)

  配置内容：

  - 场地：x ∈ [-5.0, 5.0]，y ∈ [-0.5, 8.0]
  - 圆心：O=(0,0)，半径 0.5 m
  - 圆与下边界在 (0,-0.5) 相切
  - 8 个障碍物，尺寸 0.8×0.8×1.8 m
  - 最小障碍物边缘间距：1.8 m > 0.8 m
  - 机型：gz_x500_mono_cam_down
  - 已加入 offboard_core_pkg 安装目录

  启动时使用：

cd ~/PX4-Autopilot

GZ_SIM_RESOURCE_PATH=/home/yzq/mavros_ws/simulation/worlds:/home/yzq/PX4-Autopilot/Tools/simulation/gz/models:/home/yzq/PX4-Autopilot/Tools/simulation/gz/worlds \
PX4_GZ_WORLDS=/home/yzq/mavros_ws/simulation/worlds \
PX4_GZ_MODELS=/home/yzq/PX4-Autopilot/Tools/simulation/gz/models \
PX4_GZ_WORLD=yzq_rectangular_obstacle_field \
make px4_sitl gz_x500_mono_cam_down