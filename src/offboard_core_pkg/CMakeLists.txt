cmake_minimum_required(VERSION 3.8)
project(offboard_core_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/grid.cpp
  src/mavros_iface.cpp
  src/planners/min_snap_planner.cpp
  src/planners/tsp_min_snap_planner.cpp
  src/tasks/arm_task.cpp
  src/tasks/align_down_task.cpp
  src/tasks/down_drop_task.cpp
  src/tasks/align_drop_snake_ego_task.cpp
  src/tasks/hover_task.cpp
  src/tasks/land_task.cpp
  src/tasks/presetpoint_task.cpp
  src/tasks/set_offboard_task.cpp
  src/tasks/takeoff_task.cpp
  src/tasks/goto_task.cpp
  src/tasks/snake_grid_task.cpp
  src/tasks/ego_goto_task.cpp
  src/tasks/snake_ego_avoid_task.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs mavros_msgs nav_msgs quadrotor_msgs)

if(BUILD_TESTING)
  enable_testing()
  add_executable(align_down_task_test test/align_down_task_test.cpp
    src/tasks/align_down_task.cpp)
  target_compile_features(align_down_task_test PRIVATE cxx_std_17)
  target_include_directories(align_down_task_test PRIVATE include)
  ament_target_dependencies(align_down_task_test rclcpp)
  add_test(NAME align_down_task_test COMMAND align_down_task_test)
endif()

add_executable(offboard_core_node src/nodes/offboard_core_node.cpp)
target_link_libraries(offboard_core_node ${PROJECT_NAME})
ament_target_dependencies(offboard_core_node ament_index_cpp rclcpp geometry_msgs std_msgs)

add_executable(snake_node src/nodes/snake_node.cpp)
target_link_libraries(snake_node ${PROJECT_NAME})
ament_target_dependencies(snake_node ament_index_cpp rclcpp)

add_executable(ego_goto_node src/nodes/ego_goto_node.cpp)
target_link_libraries(ego_goto_node ${PROJECT_NAME})
ament_target_dependencies(ego_goto_node ament_index_cpp rclcpp geometry_msgs nav_msgs quadrotor_msgs)

add_executable(ego_test_node src/nodes/ego_test_node.cpp)
target_link_libraries(ego_test_node ${PROJECT_NAME})
ament_target_dependencies(ego_test_node ament_index_cpp rclcpp geometry_msgs nav_msgs quadrotor_msgs)

add_executable(snake_ego_avoid_node src/nodes/snake_ego_avoid_node.cpp)
target_link_libraries(snake_ego_avoid_node ${PROJECT_NAME})
ament_target_dependencies(snake_ego_avoid_node
  ament_index_cpp rclcpp geometry_msgs nav_msgs quadrotor_msgs)

add_executable(align_drop_snake_ego_node src/nodes/snake_ego_avoid_node.cpp)
target_compile_definitions(align_drop_snake_ego_node PRIVATE ALIGN_DROP_SNAKE_EGO_NODE)
target_link_libraries(align_drop_snake_ego_node ${PROJECT_NAME})
ament_target_dependencies(align_drop_snake_ego_node
  ament_index_cpp rclcpp geometry_msgs nav_msgs quadrotor_msgs)

add_executable(lidar_to_px4_bridge src/bridge/lidar_to_px4_bridge.cpp)
ament_target_dependencies(lidar_to_px4_bridge rclcpp geometry_msgs nav_msgs)

install(TARGETS ${PROJECT_NAME} offboard_core_node snake_node ego_goto_node ego_test_node
  snake_ego_avoid_node align_drop_snake_ego_node lidar_to_px4_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../simulation/worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
ament_package()
