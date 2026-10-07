#pragma once
#include "door_navigation/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

namespace door_navigation {
template<class Stamp> double seconds(const Stamp &s) { return s.sec+s.nanosec*1e-9; }
template<class Vector> Vec3 vector(const Vector &p) { return {p.x,p.y,p.z}; }
template<class Vector> void assign(Vector &p, const Vec3 &v) { p.x=v.x(); p.y=v.y(); p.z=v.z(); }
inline geometry_msgs::msg::Point point(const Vec2 &xy, double z) {
  geometry_msgs::msg::Point p; p.x=xy.x(); p.y=xy.y(); p.z=z; return p;
}
inline geometry_msgs::msg::Quaternion quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q; q.z=std::sin(yaw/2); q.w=std::cos(yaw/2); return q;
}
inline std_msgs::msg::Header header(rclcpp::Node &node, const std::string &frame) {
  std_msgs::msg::Header h; h.frame_id=frame; h.stamp=node.now(); return h;
}
inline nav_msgs::msg::Path make_path(const Route &route, double z, double yaw, const std_msgs::msg::Header &h) {
  nav_msgs::msg::Path path; path.header=h;
  for (const auto &xy : route) {
    geometry_msgs::msg::PoseStamped p; p.header=h;
    p.pose.position=point(xy,z); p.pose.orientation=quaternion(yaw); path.poses.push_back(p);
  }
  return path;
}
}  // namespace door_navigation
