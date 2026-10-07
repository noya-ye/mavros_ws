#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ego_2d_planner_pkg/msg/polynomial2_d.hpp"
#include "ego_2d_planner_pkg/msg/emergency_stop2_d.hpp"

#include "ego_2d_planner_pkg/common/types.hpp"
#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"
#include "ego_2d_planner_pkg/plan_manage/planner_manager_2d.hpp"
#include "ego_2d_planner_pkg/diff_planner/polynomial_trajectory_2d.hpp"

using ego_2d_planner_pkg::Vec2;
using ego_2d_planner_pkg::PlannerParams2D;
using ego_2d_planner_pkg::GridMap2D;
using ego_2d_planner_pkg::PlannerManager2D;
using ego_2d_planner_pkg::PolynomialTrajectory2D;
using ego_2d_planner_pkg::msg::Polynomial2D;
using ego_2d_planner_pkg::msg::EmergencyStop2D;

class EgoReplanFSM2DNode : public rclcpp::Node
{
public:
  enum FSMExecState
  {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP
  };

  EgoReplanFSM2DNode() : Node("ego_replan_fsm_2d_node")
  {
    loadParams();

    map_.configure(p_);
    manager_.setParams(p_);

    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>("/ego_2d_planner/raw_path", 10);//a*path
    smooth_path_pub_ = create_publisher<nav_msgs::msg::Path>("/ego_2d_planner/smooth_path", 10);//MINCO path
    selected_path_pub_ = create_publisher<nav_msgs::msg::Path>("/ego_2d_planner/selected_path", 10);//selected path
    polynomial_pub_ = create_publisher<Polynomial2D>("/ego_2d_planner/polynomial_2d", 10);
    emergency_stop_pub_ = create_publisher<EmergencyStop2D>(
      "/ego_2d_planner/emergency_stop", rclcpp::QoS(1).reliable().durability_volatile());
    local_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/simple_2d_planner/local_goal", 10);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/ego_2d_planner/occupancy_grid", 1);
    cloud_2d_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ego_2d_planner/cloud_2d", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ego_2d_planner/local_goal_marker", 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      p_.cloud_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&EgoReplanFSM2DNode::cloudCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      p_.odom_topic,
      20,
      std::bind(&EgoReplanFSM2DNode::odomCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      p_.goal_topic,
      10,
      std::bind(&EgoReplanFSM2DNode::goalCallback, this, std::placeholders::_1));

    const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, p_.planning_rate));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&EgoReplanFSM2DNode::execFSMCallback, this));

    last_replan_time_ = now();
    emergency_start_time_ = now();

    RCLCPP_WARN(get_logger(), "Diff-Planner MINCO 2D FSM started");
    RCLCPP_WARN(get_logger(), "Input : %s + %s + %s",
                p_.cloud_topic.c_str(), p_.odom_topic.c_str(), p_.goal_topic.c_str());
    RCLCPP_WARN(get_logger(), "Output: /ego_2d_planner/polynomial_2d + /simple_2d_planner/local_goal");
  }

private:
  void loadParams()
  {
    p_.frame_id = declare_parameter<std::string>("frame_id", p_.frame_id);
    p_.cloud_topic = declare_parameter<std::string>("cloud_topic", p_.cloud_topic);
    p_.odom_topic = declare_parameter<std::string>("odom_topic", p_.odom_topic);
    p_.goal_topic = declare_parameter<std::string>("goal_topic", p_.goal_topic);

    p_.resolution = declare_parameter<double>("grid_map/resolution", p_.resolution);
    p_.map_size_x = declare_parameter<double>("grid_map/map_size_x", p_.map_size_x);
    p_.map_size_y = declare_parameter<double>("grid_map/map_size_y", p_.map_size_y);
    p_.cloud_min_z = declare_parameter<double>("grid_map/cloud_min_z", p_.cloud_min_z);
    p_.cloud_max_z = declare_parameter<double>("grid_map/cloud_max_z", p_.cloud_max_z);
    cloud_z_reference_ = declare_parameter<std::string>("grid_map/cloud_z_reference", "world");
    if (cloud_z_reference_ != "world" && cloud_z_reference_ != "planning_height") {
      throw std::invalid_argument("grid_map/cloud_z_reference must be world or planning_height");
    }
    if (!std::isfinite(p_.cloud_min_z) || !std::isfinite(p_.cloud_max_z) ||
        p_.cloud_min_z > p_.cloud_max_z) {
      throw std::invalid_argument("invalid grid_map/cloud_min_z or cloud_max_z");
    }
    p_.inflate_radius = declare_parameter<double>("grid_map/inflate_radius", p_.inflate_radius);
    p_.occupied_threshold = declare_parameter<int>("grid_map/occupied_threshold", p_.occupied_threshold);
    p_.persistent_cache_enable = declare_parameter<bool>("grid_map/persistent_cache_enable", p_.persistent_cache_enable);
    p_.persistent_confirm_s = declare_parameter<double>("grid_map/persistent_confirm_s", p_.persistent_confirm_s);
    p_.persistent_miss_tolerance_s = declare_parameter<double>("grid_map/persistent_miss_tolerance_s", p_.persistent_miss_tolerance_s);
    p_.persistent_min_observations = declare_parameter<int>("grid_map/persistent_min_observations", p_.persistent_min_observations);

    p_.planning_rate = declare_parameter<double>("fsm/planning_rate", p_.planning_rate);
    p_.thresh_replan_time = declare_parameter<double>("fsm/thresh_replan_time", p_.thresh_replan_time);
    p_.thresh_replan_dist = declare_parameter<double>("fsm/thresh_replan_dist", p_.thresh_replan_dist);
    p_.thresh_no_replan_dist = declare_parameter<double>("fsm/thresh_no_replan_dist", p_.thresh_no_replan_dist);
    p_.target_reached_tol = declare_parameter<double>("fsm/target_reached_tol", p_.target_reached_tol);
    p_.near_goal_dist = declare_parameter<double>("fsm/near_goal_dist", p_.near_goal_dist);
    p_.periodic_replan_dist = declare_parameter<double>("fsm/periodic_replan_dist", p_.periodic_replan_dist);
    p_.emergency_time = declare_parameter<double>("fsm/emergency_time", p_.emergency_time);
    p_.max_fsm_plan_failures = declare_parameter<int>("fsm/max_fsm_plan_failures", p_.max_fsm_plan_failures);

    p_.astar_max_iter = declare_parameter<int>("search/astar_max_iter", p_.astar_max_iter);

    p_.control_points_distance = declare_parameter<double>("manager/control_points_distance", p_.control_points_distance);
    p_.bspline_sample_step = declare_parameter<double>("manager/bspline_sample_step", p_.bspline_sample_step);
    p_.collision_check_step = declare_parameter<double>("manager/collision_check_step", p_.collision_check_step);
    p_.fallback_to_raw_path = declare_parameter<bool>("manager/fallback_to_raw_path", p_.fallback_to_raw_path);
    p_.hover_if_plan_failed = declare_parameter<bool>("manager/hover_if_plan_failed", p_.hover_if_plan_failed);
    p_.lookahead_dist = declare_parameter<double>("manager/lookahead_dist", p_.lookahead_dist);
    p_.fixed_z = declare_parameter<double>("manager/fixed_z", p_.fixed_z);

    p_.optimizer_enable = declare_parameter<bool>("optimization/enable", p_.optimizer_enable);
    p_.optimizer_max_iter = declare_parameter<int>("optimization/max_iter", p_.optimizer_max_iter);
    p_.optimizer_step_size = declare_parameter<double>("optimization/step_size", p_.optimizer_step_size);
    p_.optimizer_max_update = declare_parameter<double>("optimization/max_update", p_.optimizer_max_update);
    p_.dist0 = declare_parameter<double>("optimization/dist0", p_.dist0);
    p_.max_vel = declare_parameter<double>("optimization/max_vel", p_.max_vel);
    p_.max_acc = declare_parameter<double>("optimization/max_acc", p_.max_acc);
    p_.knot_span = declare_parameter<double>("optimization/knot_span", p_.knot_span); // legacy compatibility only
    p_.lambda_smooth = declare_parameter<double>("optimization/lambda_smooth", p_.lambda_smooth);
    p_.lambda_collision = declare_parameter<double>("optimization/lambda_collision", p_.lambda_collision);
    p_.lambda_feasibility = declare_parameter<double>("optimization/lambda_feasibility", p_.lambda_feasibility);
    p_.lambda_fitness = declare_parameter<double>("optimization/lambda_fitness", p_.lambda_fitness);
    p_.max_rebound_attempts = declare_parameter<int>("optimization/max_rebound_attempts", p_.max_rebound_attempts);//优化尝试次数
    p_.retry_collision_scale = declare_parameter<double>("optimization/retry_collision_scale", p_.retry_collision_scale);
    p_.retry_smooth_scale = declare_parameter<double>("optimization/retry_smooth_scale", p_.retry_smooth_scale);
    p_.retry_dist0_scale = declare_parameter<double>("optimization/retry_dist0_scale", p_.retry_dist0_scale);
    p_.diff_piece_length = declare_parameter<double>("diff/piece_length", p_.diff_piece_length);
    p_.diff_samples_per_piece = declare_parameter<int>("diff/samples_per_piece", p_.diff_samples_per_piece);
    p_.diff_max_evaluations = declare_parameter<int>("diff/max_evaluations", p_.diff_max_evaluations);
    p_.diff_max_solve_ms = declare_parameter<double>("diff/max_solve_ms", p_.diff_max_solve_ms);
    p_.diff_weight_time = declare_parameter<double>("diff/weight_time", p_.diff_weight_time);
    p_.diff_weight_collision = declare_parameter<double>("diff/weight_collision", p_.diff_weight_collision);
    p_.diff_weight_feasibility = declare_parameter<double>("diff/weight_feasibility", p_.diff_weight_feasibility);
    p_.max_tracking_error = declare_parameter<double>("fsm/max_tracking_error", p_.max_tracking_error);
  }

  std::string stateName(FSMExecState s) const
  {
    switch (s) {
      case INIT: return "INIT";
      case WAIT_TARGET: return "WAIT_TARGET";
      case GEN_NEW_TRAJ: return "GEN_NEW_TRAJ";
      case REPLAN_TRAJ: return "REPLAN_TRAJ";
      case EXEC_TRAJ: return "EXEC_TRAJ";
      case EMERGENCY_STOP: return "EMERGENCY_STOP";
      default: return "UNKNOWN";
    }
  }

  void changeFSMExecState(FSMExecState new_state, const std::string& reason)
  {
    if (exec_state_ == new_state) {
      return;
    }

    RCLCPP_WARN(get_logger(), "[FSM] %s -> %s, reason=%s",
                stateName(exec_state_).c_str(), stateName(new_state).c_str(), reason.c_str());

    exec_state_ = new_state;
    if (new_state == EMERGENCY_STOP) {
      emergency_start_time_ = now();
      if (!emergency_stop_active_) {
        emergency_stop_msg_.header.stamp = emergency_start_time_;
        emergency_stop_msg_.header.frame_id = p_.frame_id;
        emergency_stop_msg_.position.x = current_pos_.x;
        emergency_stop_msg_.position.y = current_pos_.y;
        emergency_stop_msg_.position.z = current_z_;
        emergency_stop_active_ = true;
      }
      selected_path_.clear();
      selected_path_dirty_ = true;
      have_exec_path_ = false;
      active_trajectory_.pieces.clear();
      active_duration_ = 0.0;
      emergency_stop_pub_->publish(emergency_stop_msg_);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto& position = msg->pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[ODOM] reject non-finite position");
      return;
    }
    current_pos_ = Vec2{msg->pose.pose.position.x, msg->pose.pose.position.y};
    current_z_ = msg->pose.pose.position.z;
    if (!have_yaw_) {
      const auto& q = msg->pose.pose.orientation;
      const double squared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (std::isfinite(squared) && squared > 1e-12) {
        locked_yaw_ = std::atan2(2.0*(q.w*q.z + q.x*q.y), squared - 2.0*(q.y*q.y + q.z*q.z));
        have_yaw_ = true;
      }
    }
    have_odom_ = true;
  }

  void activeTrajectoryState(Vec2 &p, Vec2 &v, Vec2 &a) const
  {
    p = current_pos_; v = {}; a = {};
    if (active_trajectory_.pieces.empty()) return;
    const double t = std::clamp((planning_start_time_ - active_start_time_).seconds(), 0.0, active_duration_);
    p = active_trajectory_.evaluate(t);
    v = active_trajectory_.evaluate(t, 1);
    a = active_trajectory_.evaluate(t, 2);
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const Vec2 new_goal{msg->pose.position.x, msg->pose.position.y};
    if (!ego_2d_planner_pkg::finite(new_goal)) return;

    if (!have_goal_ || ego_2d_planner_pkg::dist(new_goal, goal_pos_) > 1e-3) {
      goal_pos_ = new_goal;
      have_goal_ = true;
      have_new_target_ = true;
      plan_fail_count_ = 0;

      RCLCPP_WARN(get_logger(), "[GOAL] new target x=%.2f y=%.2f", goal_pos_.x, goal_pos_.y);

      if (exec_state_ == WAIT_TARGET || exec_state_ == EXEC_TRAJ || exec_state_ == EMERGENCY_STOP) {
        changeFSMExecState(GEN_NEW_TRAJ, "TARGET");
      }
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_odom_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[MAP] waiting odom before building local grid");
      return;
    }

    std::size_t used = 0;
    try {
      // Validate before clearing the last complete map. The iterators below require FLOAT32 XYZ.
      for (const auto* name : {"x", "y", "z"}) {
        const auto field = std::find_if(msg->fields.begin(), msg->fields.end(),
          [name](const auto& f) { return f.name == name; });
        if (field == msg->fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            field->count != 1 || field->offset > msg->point_step ||
            msg->point_step - field->offset < sizeof(float)) {
          throw std::invalid_argument("PointCloud2 requires valid FLOAT32 x/y/z fields");
        }
      }
      if (static_cast<std::size_t>(msg->width) * msg->point_step != msg->row_step ||
          static_cast<std::size_t>(msg->height) * msg->row_step != msg->data.size() ||
          msg->is_bigendian) {
        throw std::invalid_argument("PointCloud2 requires packed little-endian rows");
      }
      const double reference_z = cloud_z_reference_ == "planning_height" ? p_.fixed_z : 0.0;
      const double min_z = reference_z + p_.cloud_min_z;
      const double max_z = reference_z + p_.cloud_max_z;
      map_.resetAround(current_pos_);
      map_.beginUpdate(now().seconds());
      if (!msg->data.empty()) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
          const double x = static_cast<double>(*iter_x);
          const double y = static_cast<double>(*iter_y);
          const double z = static_cast<double>(*iter_z);
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
          if (z < min_z || z > max_z) continue;
          map_.setOccupiedWorld(Vec2{x, y});
          ++used;
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "[MAP] PointCloud2 iterator error: %s", e.what());
      return;
    }//在点云中按照字段名构造x，y,z的迭代器，遍历点云，判断点是否有效，是否在z范围内，如果有效则将其设置为占用栅格
    //used记录了有效点的数量

    map_.finishUpdate();
    map_.inflateObstacles();
    // Diff's base-point/direction constraints do not require an ESDF.
    have_map_ = true;
    publishGrid();
    publishCloud2D(msg->header);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[MAP] cloud used=%zu grid=%dx%d res=%.2f origin=(%.2f %.2f) cache=%zu/%zu",
      used, map_.width(), map_.height(), map_.resolution(), map_.origin_x(), map_.origin_y(),
      map_.persistentCellCount(), map_.persistentCandidateCount());
  }
//ego主程序
  void execFSMCallback()
  {
    // Keep cancellation alive through failed retries and newly received goals.
    // Reusing the issue stamp makes repetitions idempotent at the trajectory server.
    if (emergency_stop_active_) emergency_stop_pub_->publish(emergency_stop_msg_);
    // Check the currently executable plan also while a new goal or retry is pending.
    // A failed replacement must not leave unsafe old motion running until the retry limit.
    if (have_exec_path_ && have_odom_ && have_map_ && !active_trajectory_.pieces.empty()) {
      const double executed_time = std::clamp((now() - active_start_time_).seconds(), 0.0, active_duration_);
      if (map_.isOccupiedWorld(current_pos_) || !active_trajectory_.collisionFree(map_, executed_time) ||
          ego_2d_planner_pkg::dist(active_trajectory_.evaluate(executed_time), current_pos_) > p_.max_tracking_error)
        changeFSMExecState(EMERGENCY_STOP, "TRAJ_CHECK");
    }
    switch (exec_state_) {
      case INIT:
      {
        if (!have_odom_ || !have_map_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                               "[FSM][INIT] waiting odom=%d map=%d", have_odom_, have_map_);
          return;
        }
        changeFSMExecState(WAIT_TARGET, "INIT_OK");
        return;
      }

      case WAIT_TARGET:
      {
        if (!have_goal_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[FSM][WAIT_TARGET] waiting goal");
          return;
        }
        changeFSMExecState(GEN_NEW_TRAJ, "HAVE_TARGET");
        return;
      }

      case GEN_NEW_TRAJ:
      case REPLAN_TRAJ:
      {
        if (!readyToPlan()) return;
        planning_start_time_ = now() + rclcpp::Duration::from_seconds(p_.diff_max_solve_ms * 0.001 + 0.020);
        Vec2 start_p, start_v, start_a;
        activeTrajectoryState(start_p, start_v, start_a);
        const bool initial = exec_state_ == GEN_NEW_TRAJ;
        const auto result = manager_.reboundReplan(map_, start_p, goal_pos_, initial, start_v, start_a);
        handlePlanResult(result, initial);
        return;
      }

      case EXEC_TRAJ:
      {
        execTrajCallback();
        return;
      }

      case EMERGENCY_STOP:
      {
        emergencyStopCallback();
        return;
      }
    }
  }

  bool readyToPlan() 
  {
    if (!have_odom_ || !have_map_ || !have_goal_ || !have_yaw_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[PLAN] waiting odom=%d map=%d goal=%d", have_odom_, have_map_, have_goal_);
      return false;
    }
    return true;
  }

  void handlePlanResult(const PlannerManager2D::PlanResult& result, bool init)
  {
    publishPath(result.raw_path, raw_path_pub_);
    publishPath(result.smooth_path, smooth_path_pub_);

    if (result.success) {
      if (!publishPolynomial2D(result)) {
        changeFSMExecState(EMERGENCY_STOP, "PLAN_TIMING_OVERRUN");
        return;
      }
      selected_path_ = result.selected_path;
      selected_path_dirty_ = true;
      have_exec_path_ = !selected_path_.empty();
      have_new_target_ = false;
      plan_fail_count_ = 0;
      last_replan_time_ = now();

      publishPath(selected_path_, selected_path_pub_);
      active_trajectory_ = result.trajectory;
      active_duration_ = result.trajectory.duration();
      active_start_time_ = planning_start_time_;

      RCLCPP_WARN(get_logger(),
                  "[PLAN_OK] %s raw=%zu smooth=%zu attempt=%d cost=%.2f->%.2f",
                  result.message.c_str(), result.raw_path.size(), result.smooth_path.size(),
                  result.rebound_attempt, result.init_cost, result.final_cost);

      changeFSMExecState(EXEC_TRAJ, init ? "GEN_NEW_SUCCESS" : "REPLAN_SUCCESS");
      return;
    }

    ++plan_fail_count_;
    RCLCPP_WARN(get_logger(),
                "[PLAN_FAIL] state=%s fail_count=%d/%d msg=%s raw_safe=%d smooth_safe=%d",
                init ? "GEN_NEW_TRAJ" : "REPLAN_TRAJ",
                plan_fail_count_, p_.max_fsm_plan_failures,
                result.message.c_str(), result.raw_path_safe, result.smooth_safe);

    // Failed optimized trajectories are never sent to the controller.
    if (plan_fail_count_ >= p_.max_fsm_plan_failures) {
      changeFSMExecState(EMERGENCY_STOP, "PLAN_FAIL_LIMIT");
    } else {
      changeFSMExecState(init ? GEN_NEW_TRAJ : REPLAN_TRAJ, "PLAN_RETRY");
    }
  }

  void execTrajCallback()
  {
    if (!readyToPlan()) return;

    if (have_new_target_) {
      changeFSMExecState(GEN_NEW_TRAJ, "NEW_TARGET_DURING_EXEC");
      return;
    }

    if (!have_exec_path_ || selected_path_.size() < 2) {
      changeFSMExecState(GEN_NEW_TRAJ, "EMPTY_EXEC_PATH");
      return;
    }

    const double remain_dist = ego_2d_planner_pkg::dist(current_pos_, goal_pos_);
    if (remain_dist <= p_.target_reached_tol) {
      holdGoal();
      return;
    }

    // The already validated MINCO trajectory ends at rest, also for short goals.
    if (remain_dist <= p_.near_goal_dist) {
      return;
    }

    const double time_since_replan = (now() - last_replan_time_).seconds();
    if (time_since_replan > p_.thresh_replan_time && remain_dist > p_.periodic_replan_dist) {
      changeFSMExecState(REPLAN_TRAJ, "PERIODIC_REPLAN");
      return;
    }

    const Vec2 local_goal = ego_2d_planner_pkg::lookaheadPoint2D(selected_path_, current_pos_, p_.lookahead_dist);
    publishLocalGoal(local_goal);
    publishMarker(local_goal);
    publishPath(selected_path_, selected_path_pub_);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "[EXEC] local_goal=(%.2f %.2f) cur=(%.2f %.2f) remain=%.2f",
                         local_goal.x, local_goal.y, current_pos_.x, current_pos_.y, remain_dist);
  }

  void holdGoal()
  {
    publishLocalGoal(goal_pos_);
    publishMarker(goal_pos_);
  }

  void emergencyStopCallback()
  {
    // The cancellation topic is unconditional; this option only controls the legacy local goal.
    if (p_.hover_if_plan_failed && have_odom_) {
      const Vec2 hold{emergency_stop_msg_.position.x, emergency_stop_msg_.position.y};
      publishLocalGoal(hold);
      publishMarker(hold);
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "[EMERGENCY_STOP] hover at current position; wait %.2fs then try replan",
                         p_.emergency_time);

    if (have_goal_ && have_map_ && have_odom_ &&
        (now() - emergency_start_time_).seconds() > p_.emergency_time) {
      plan_fail_count_ = 0;
      changeFSMExecState(GEN_NEW_TRAJ, "EMERGENCY_RETRY");
    }
  }


  bool publishPolynomial2D(const PlannerManager2D::PlanResult& result)
  {
    if (!result.success || !result.trajectory.valid() || now() >= planning_start_time_) return false;
    Polynomial2D msg;
    msg.header.stamp = now();
    if (emergency_stop_active_ &&
        rclcpp::Time(msg.header.stamp) <= rclcpp::Time(emergency_stop_msg_.header.stamp)) return false;
    msg.header.frame_id = p_.frame_id;
    msg.start_time = planning_start_time_;
    msg.traj_id = ++polynomial_traj_id_;
    msg.order = 5; msg.fixed_z = p_.fixed_z;
    for (const auto& piece : result.trajectory.pieces) {
      msg.durations.push_back(piece.duration);
      for (const auto& coefficient : piece.coefficients) {
        msg.coeff_x.push_back(coefficient.x); msg.coeff_y.push_back(coefficient.y);
      }
    }
    polynomial_pub_->publish(msg);
    emergency_stop_active_ = false;
    RCLCPP_WARN(get_logger(), "[POLYNOMIAL_2D] id=%d pieces=%zu duration=%.2f solve_ms=%.2f eval=%d",
      msg.traj_id, msg.durations.size(), result.trajectory.duration(),
      result.planning_ms, result.optimizer_evaluations);
    return true;
  }

  void publishGrid()
  {
    if (grid_pub_->get_subscription_count() == 0 &&
        grid_pub_->get_intra_process_subscription_count() == 0) return;
    auto& msg = grid_msg_;
    msg.header.stamp = now();
    msg.header.frame_id = p_.frame_id;
    msg.info.resolution = static_cast<float>(map_.resolution());
    msg.info.width = static_cast<uint32_t>(map_.width());
    msg.info.height = static_cast<uint32_t>(map_.height());
    msg.info.origin.position.x = map_.origin_x();
    msg.info.origin.position.y = map_.origin_y();
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = map_.data();
    grid_pub_->publish(msg);
  }

  void publishCloud2D(const std_msgs::msg::Header& header)
  {
    if (cloud_2d_pub_->get_subscription_count() == 0 &&
        cloud_2d_pub_->get_intra_process_subscription_count() == 0) return;
    auto& msg = cloud_2d_msg_;
    msg.header = header;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    if (msg.fields.empty()) modifier.setPointCloud2FieldsByString(1, "xyz");
    const auto& raw = map_.rawData();
    modifier.resize(std::count_if(raw.begin(), raw.end(),
      [this](int8_t value) { return value >= p_.occupied_threshold; }));
    if (msg.data.empty()) {
      cloud_2d_pub_->publish(msg);
      return;
    }
    sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    for (int iy = 0; iy < map_.height(); ++iy) {
      for (int ix = 0; ix < map_.width(); ++ix) {
        if (raw[map_.index(ix, iy)] < p_.occupied_threshold) continue;
        const auto point = map_.gridToWorld(ix, iy);
        *x = static_cast<float>(point.x);
        *y = static_cast<float>(point.y);
        *z = static_cast<float>(p_.fixed_z);
        ++x; ++y; ++z;
      }
    }
    cloud_2d_pub_->publish(msg);
  }

  void publishPath(const std::vector<Vec2>& path, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub)
  {
    if (pub->get_subscription_count() == 0 && pub->get_intra_process_subscription_count() == 0) return;
    const bool selected = pub == selected_path_pub_;
    auto& msg = selected ? selected_path_msg_ : (pub == raw_path_pub_ ? raw_path_msg_ : smooth_path_msg_);
    msg.header.stamp = now();
    msg.header.frame_id = p_.frame_id;
    if (!selected || selected_path_dirty_) {
      msg.poses.resize(path.size());
      for (std::size_t i = 0; i < path.size(); ++i) {
        auto& ps = msg.poses[i];
        ps.pose.position.x = path[i].x;
        ps.pose.position.y = path[i].y;
        ps.pose.position.z = p_.fixed_z;
        ps.pose.orientation.z = std::sin(locked_yaw_ * 0.5);
        ps.pose.orientation.w = std::cos(locked_yaw_ * 0.5);
      }
      if (selected) selected_path_dirty_ = false;
    }
    for (auto& ps : msg.poses) {
      ps.header = msg.header;
    }
    pub->publish(msg);
  }

  void publishLocalGoal(const Vec2& p)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = p_.frame_id;
    msg.pose.position.x = p.x;
    msg.pose.position.y = p.y;
    msg.pose.position.z = p_.fixed_z;
    msg.pose.orientation.z = std::sin(locked_yaw_ * 0.5);
    msg.pose.orientation.w = std::cos(locked_yaw_ * 0.5);
    local_goal_pub_->publish(msg);
  }

  void publishMarker(const Vec2& p)
  {
    if (marker_pub_->get_subscription_count() == 0 &&
        marker_pub_->get_intra_process_subscription_count() == 0) return;
    auto& m = marker_msg_;
    m.header.stamp = now();
    m.header.frame_id = p_.frame_id;
    m.ns = "ego_2d_planner";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = p_.fixed_z;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.18;
    m.scale.y = 0.18;
    m.scale.z = 0.18;
    m.color.r = 0.1f;
    m.color.g = 1.0f;
    m.color.b = 0.2f;
    m.color.a = 1.0f;
    marker_pub_->publish(m);
  }

private:
  PlannerParams2D p_;
  GridMap2D map_;
  PlannerManager2D manager_;

  FSMExecState exec_state_{INIT};

  bool have_odom_{false};
  bool have_map_{false};
  bool have_goal_{false};
  bool have_new_target_{false};
  bool have_exec_path_{false};
  

  Vec2 current_pos_;
  double current_z_{0.0};
  std::string cloud_z_reference_{"world"};
  Vec2 goal_pos_;

  std::vector<Vec2> selected_path_;
  PolynomialTrajectory2D active_trajectory_;
  bool have_yaw_{false};
  double locked_yaw_{0.0};
  rclcpp::Time planning_start_time_;
  bool selected_path_dirty_{true};
  bool emergency_stop_active_{false};
  EmergencyStop2D emergency_stop_msg_;
  nav_msgs::msg::OccupancyGrid grid_msg_;
  sensor_msgs::msg::PointCloud2 cloud_2d_msg_;
  nav_msgs::msg::Path raw_path_msg_, smooth_path_msg_, selected_path_msg_;
  visualization_msgs::msg::Marker marker_msg_;
  double active_duration_{0.0};
  rclcpp::Time active_start_time_;
  int plan_fail_count_{0};
  int polynomial_traj_id_{0};

  rclcpp::Time last_replan_time_;
  rclcpp::Time emergency_start_time_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_path_pub_;
  rclcpp::Publisher<Polynomial2D>::SharedPtr polynomial_pub_;
  rclcpp::Publisher<EmergencyStop2D>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_2d_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoReplanFSM2DNode>());
  rclcpp::shutdown();
  return 0;
}
