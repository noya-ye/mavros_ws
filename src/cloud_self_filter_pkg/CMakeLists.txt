cmake_minimum_required(VERSION 3.8)
project(cloud_self_filter_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(cloud_self_filter_node
  src/cloud_self_filter_node.cpp
)

ament_target_dependencies(cloud_self_filter_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(cloud_self_filter_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  cloud_self_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
