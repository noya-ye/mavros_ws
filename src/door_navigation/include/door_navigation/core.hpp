#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <array>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace door_navigation {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Route = std::vector<Vec2>;
constexpr double pi = 3.14159265358979323846;
constexpr double neg_inf = -std::numeric_limits<double>::infinity();

struct Config {
  double resolution = 0.02, x_min = -1.0, x_max = 4.0, y_min = -1.5, y_max = 1.5;
  double radius = 0.25, margin = 0.0;
  double corridor_width = 1.5, corridor_tolerance = 0.20, door_min = 0.65, door_max = 0.95;
  double pre_distance = 0.50, post_distance = 0.50, wall_thickness = 0.15;
  double max_speed = 0.30, max_acceleration = 0.30, sample_dt = 0.05;
  void validate() const;
};
struct Gate {
  Vec2 center, normal;
  double width;
  std::array<Vec2, 2> edges;
};
struct Layers { cv::Mat1b obstacles, unknown, blocked; };
class Grid {
 public:
  explicit Grid(const Config &config);
  Config cfg;
  cv::Mat1b occupied, observed;
  cv::Point cell(const Vec2 &p) const;
  Vec2 point(double x, double y) const;
  bool inside(cv::Point p) const;
  void add(const Vec2 &point, const Vec2 &origin);
  void occupy(const Vec2 &point);
  Layers inflation_layers() const;
};
std::vector<Gate> detect_gates(const Grid &grid);
double corridor_heading(const Route &points);
bool clear_segment(const Grid &grid, const cv::Mat1b &blocked, const Vec2 &a, const Vec2 &b);
Route astar(const Grid &grid, const cv::Mat1b &blocked, const Vec2 &start, const Vec2 &goal);
Route fixed_gate_route(const Config &config, const Gate &gate);
std::string straight_segment_check(const Grid &grid, const Layers &layers, const Vec2 &a, const Vec2 &b);
Route plan(const Grid &grid, const cv::Mat1b &blocked, const Gate &gate);

struct Sample {
  double t = 0;
  Vec3 p = Vec3::Zero(), v = Vec3::Zero(), a = Vec3::Zero();
};
using Trajectory = std::vector<Sample>;
Trajectory trajectory(const Route &route, const Config &cfg);
Sample interpolate(const Sample &left, const Sample &right, double t);
Eigen::Matrix2d basis(double heading);
Vec2 world_xy(const Vec2 &local, const Vec3 &reference, double heading);
Vec2 grid_origin(const Config &cfg, const Vec3 &reference, double heading);
std::vector<int8_t> raw_map(const Grid &grid);

class CommandStream {
 public:
  explicit CommandStream(std::string frame, double status_age = 0.6, double start_age = 0.3);
  bool enabled = false;
  double yaw = 0;
  uint32_t trajectory_id = 0;
  void clear();
  void enable(bool value);
  void load(const std::string &frame, double start, const Trajectory &rows, double yaw, double now);
  void update_status(double stamp, bool valid);
  std::optional<Sample> sample(double now);
 private:
  std::string frame_;
  double max_status_age_, max_start_age_;
  Trajectory rows_;
  double start_ = neg_inf, last_start_ = neg_inf, status_stamp_ = neg_inf;
  bool status_valid_ = false;
};
}  // namespace door_navigation
