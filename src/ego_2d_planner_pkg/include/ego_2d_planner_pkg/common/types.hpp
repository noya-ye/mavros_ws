#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

namespace ego_2d_planner_pkg
{

struct Vec2
{
  double x{0.0};
  double y{0.0};

  Vec2() = default;
  Vec2(double _x, double _y) : x(_x), y(_y) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double s) const { return {x * s, y * s}; }
  Vec2 operator/(double s) const { return {x / s, y / s}; }
  Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
  Vec2& operator*=(double s) { x *= s; y *= s; return *this; }
};

inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline double norm2(const Vec2& a) { return dot(a, a); }
inline double norm(const Vec2& a) { return std::sqrt(norm2(a)); }
inline double dist(const Vec2& a, const Vec2& b) { return norm(a - b); }
inline bool finite(const Vec2& p) { return std::isfinite(p.x) && std::isfinite(p.y); }

inline Vec2 normalized(const Vec2& v)
{
  const double n = norm(v);
  if (n < 1e-9) return {0.0, 0.0};
  return v / n;
}

inline Vec2 clampNorm(const Vec2& v, double max_norm)
{
  const double n = norm(v);
  if (n <= max_norm || n < 1e-9) return v;
  return v * (max_norm / n);
}

struct PlannerParams2D
{
  std::string frame_id{"camera_init"};
  std::string cloud_topic{"/cloud_registered_filtered"};
  std::string odom_topic{"/Odometry"};
  std::string goal_topic{"/simple_2d_planner/goal"};

  double resolution{0.10};
  double map_size_x{12.0};
  double map_size_y{12.0};
  double cloud_min_z{-0.30};
  double cloud_max_z{2.20};
  double inflate_radius{0.30};
  int occupied_threshold{50};

  // Persistent occupied cache. A world-grid cell that is observed as occupied
  // for a continuous time window becomes permanently occupied during this node's lifetime.
  bool persistent_cache_enable{false};
  double persistent_confirm_s{2.0};
  double persistent_miss_tolerance_s{0.35};
  int persistent_min_observations{3};

  double planning_rate{10.0};

  // EGO-style FSM/replan parameters
  double thresh_replan_time{0.80};
  double thresh_replan_dist{0.80};
  double thresh_no_replan_dist{0.35};  // Legacy compatibility parameter.
  double target_reached_tol{0.25};
  double near_goal_dist{0.80};
  double periodic_replan_dist{0.80};
  double emergency_time{0.80};
  int max_fsm_plan_failures{3};

  int astar_max_iter{30000};
  double control_points_distance{0.25};
  double bspline_sample_step{0.05};
  double collision_check_step{0.05};
  bool fallback_to_raw_path{false};  // EGO-style: raw path is NOT accepted as normal success
  bool hover_if_plan_failed{true};
  double lookahead_dist{0.70};
  double fixed_z{1.0};

  bool optimizer_enable{true};
  int optimizer_max_iter{80};
  double optimizer_step_size{0.035};
  double optimizer_max_update{0.05};
  double dist0{0.45};
  double max_vel{0.8};
  double max_acc{0.8};
  double knot_span{0.25};

  double lambda_smooth{1.0};
  double lambda_collision{3.0};
  double lambda_feasibility{0.2};
  double lambda_fitness{0.6};

  // Rebound-style retry parameters. Each retry gets more collision-aware
  // and less over-smoothed, similar to EGO's rebound/replan spirit.
  int max_rebound_attempts{4};
  double retry_collision_scale{1.6};
  double retry_smooth_scale{0.75};
  double retry_dist0_scale{1.08};

  // Diff-Planner: planar MINCO waypoints and independent piece durations.
  double diff_piece_length{0.50};
  int diff_samples_per_piece{16};
  int diff_max_evaluations{240};
  double diff_max_solve_ms{60.0};
  double diff_weight_time{0.20};
  double diff_weight_collision{2000.0};
  double diff_weight_feasibility{100.0};
  double max_tracking_error{0.50};
};

}  // namespace ego_2d_planner_pkg
