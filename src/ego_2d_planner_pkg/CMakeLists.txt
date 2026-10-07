cmake_minimum_required(VERSION 3.8)
project(ego_2d_planner_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline2D.msg"
  "msg/EmergencyStop2D.msg"
  "msg/Polynomial2D.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
)

include_directories(include)

add_library(ego_2d_planner_core
  src/plan_env/grid_map_2d.cpp
  src/path_searching/astar_2d.cpp
  src/diff_planner/polynomial_trajectory_2d.cpp
  src/diff_planner/diff_optimizer_2d.cpp
  src/plan_manage/planner_manager_2d.cpp
)

target_compile_features(ego_2d_planner_core PUBLIC cxx_std_17)
target_link_libraries(ego_2d_planner_core Eigen3::Eigen)

ament_target_dependencies(ego_2d_planner_core
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

add_executable(ego_replan_fsm_2d_node
  src/plan_manage/ego_replan_fsm_2d_node.cpp
)

target_compile_features(ego_replan_fsm_2d_node PUBLIC cxx_std_17)
target_link_libraries(ego_replan_fsm_2d_node ego_2d_planner_core)

ament_target_dependencies(ego_replan_fsm_2d_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  builtin_interfaces
)

add_executable(traj_server_2d_node
  src/plan_manage/traj_server_2d_node.cpp
)

target_compile_features(traj_server_2d_node PUBLIC cxx_std_17)
target_link_libraries(traj_server_2d_node ego_2d_planner_core)

ament_target_dependencies(traj_server_2d_node
  rclcpp
  nav_msgs
  quadrotor_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
)

# Link executables that use this package's generated message types.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ego_replan_fsm_2d_node "${cpp_typesupport_target}")
target_link_libraries(traj_server_2d_node "${cpp_typesupport_target}")

install(TARGETS
  ego_2d_planner_core
  ego_replan_fsm_2d_node
  traj_server_2d_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  rviz
  third_party
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ego_2d_planner_core)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  quadrotor_msgs
  std_msgs
  builtin_interfaces
  rosidl_default_runtime
  Eigen3
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_diff_core test/test_diff_core.cpp)
  target_compile_features(test_diff_core PUBLIC cxx_std_17)
  target_link_libraries(test_diff_core ego_2d_planner_core)
  add_test(NAME diff_core COMMAND test_diff_core)
endif()

ament_package()
