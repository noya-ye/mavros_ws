// 正常执行蛇形遍历。
//   - 订阅 EGO 的膨胀地图：

//     /ego_2d_planner/occupancy_grid

//   - 当前蛇形航段距离障碍物小于可调参数时触发 EGO。
//   - 当前目标格被膨胀地图占据时自动跳过。
//   - EGO 目标使用后续第一个未占据蛇形航点。
//   - EGO 成功后，从该航点之后继续蛇形遍历。
//   - EGO 失败、超时或没有安全航点时进入 LandTask。
//   - 支持多个静态障碍物。
//   - 地图超时、占据阈值、避障触发距离均可配置。
//   - 增加了绝对 ENU 目标接口：
// SnakeGridTask
//     -> EGO 避障
//     -> EGO 成功
//     -> SnakeGridTask 继续
//     -> 所有蛇形航点完成
//     -> SnakeEgoAvoidTask 成功退出
#include "offboard_core_pkg/tasks/snake_ego_avoid_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "offboard_core_pkg/mavros_iface.hpp"

namespace offboard_core_pkg {
namespace {

std::uint64_t nowUs(const rclcpp::Clock::SharedPtr &clock) {
  return static_cast<std::uint64_t>(clock->now().nanoseconds() / 1000ULL);
}

double pointSegmentDistance(
  double px, double py, double x0, double y0, double x1, double y1) {
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq < 1e-9) return std::hypot(px - x0, py - y0);

  const double t = std::clamp(
    ((px - x0) * dx + (py - y0) * dy) / length_sq, 0.0, 1.0);
  return std::hypot(px - (x0 + t * dx), py - (y0 + t * dy));
}
//计算二维平面上一个点 \(P(px,py)\) 到线段 \(AB\) 的最短距离
}  // namespace

SnakeEgoAvoidTask::SnakeEgoAvoidTask(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
  const Config &cfg)
: logger_(logger),
  clock_(clock),
  cfg_(cfg),
  snake_(logger, cfg.snake),
  ego_(logger, clock, goal_pub, cfg.ego),
  land_(cfg.landing_timeout_s, cfg.landing_retry_interval_s) {}

std::string SnakeEgoAvoidTask::name() const { return "SNAKE_EGO_AVOID"; }

const char *SnakeEgoAvoidTask::phaseName() const {
  switch (phase_) {
    case Phase::SNAKE: return "SNAKE";
    case Phase::AVOIDING: return "AVOIDING";
    case Phase::LANDING: return "LANDING";
    case Phase::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

void SnakeEgoAvoidTask::onEnter(Context &ctx, MavrosIface &iface) {
  phase_ = Phase::SNAKE;
  avoidance_target_index_ = 0;
  avoidance_elapsed_s_ = 0.0;
  failure_reason_.clear();
  snake_.onEnter(ctx, iface);
  if (snake_.failed()) {
    phase_ = Phase::FAILED;
    ctx.fault = "snake route initialization failed";
    RCLCPP_ERROR(logger_, "[SNAKE_EGO] phase=%s: %s", phaseName(),
                 ctx.fault.c_str());
    return;
  }
  RCLCPP_INFO(
    logger_,
    "[SNAKE_EGO] phase=%s started: aircraft=(%.2f, %.2f, %.2f), "
    "grid=%dx%d, cell_size=%.2f m, first_axis=%s",
    phaseName(), ctx.position_enu.x, ctx.position_enu.y, ctx.position_enu.z,
    cfg_.snake.x_cells, cfg_.snake.y_cells, cfg_.snake.cell_size,
    cfg_.snake.first_axis == SnakeGridTask::FirstAxis::X_FIRST ? "x" : "y");
}

ITask::Status SnakeEgoAvoidTask::tick(
  Context &ctx, MavrosIface &iface, double dt_s) {
  const double dt = std::clamp(std::isfinite(dt_s) ? dt_s : 0.0, 0.0, 0.2);

  if (phase_ == Phase::FAILED) return Status::FAILURE;
  if (phase_ == Phase::LANDING) return tickLanding(ctx, iface, dt);



  //蛇形遍历
  if (phase_ == Phase::SNAKE) {
    if (snake_.finished()) {
      RCLCPP_INFO(logger_, "[SNAKE_EGO] phase=%s: snake route complete",
                  phaseName());
      return Status::SUCCESS;
    }

    // Skip every currently targeted cell that is occupied in the inflated map.
    SnakeGridTask::WaypointInfo waypoint;
    while (snake_.waypointAt(snake_.currentIndex(), waypoint) &&
           occupiedAt(ctx, waypoint.x, waypoint.y)) {
      RCLCPP_WARN(
        logger_,
        "[SNAKE_EGO] phase=%s: skip occupied cell=(%d, %d), "
        "target=(%.2f, %.2f, %.2f)",
        phaseName(), waypoint.ix, waypoint.iy, waypoint.x, waypoint.y,
        waypoint.z);
      snake_.skipCurrentWaypoint();
    }//如果当前蛇形航点被膨胀地图占据，则跳过该航点，继续下一个航点
    if (snake_.finished()) {
      RCLCPP_INFO(logger_, "[SNAKE_EGO] phase=%s: all remaining cells occupied; "
                  "snake route complete", phaseName());
      return Status::SUCCESS;
    }
    if (!snake_.waypointAt(snake_.currentIndex(), waypoint)) {
      ctx.fault = "snake waypoint index is invalid";
      RCLCPP_ERROR(logger_, "[SNAKE_EGO] phase=%s: %s", phaseName(),
                   ctx.fault.c_str());
      return Status::FAILURE;
    }

    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[SNAKE_EGO] phase=%s cell=(%d, %d) [%zu/%zu], "
      "aircraft=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
      phaseName(), waypoint.ix, waypoint.iy,
      snake_.currentIndex() + 1, snake_.totalWaypoints(),
      ctx.position_enu.x, ctx.position_enu.y, ctx.position_enu.z,
      waypoint.x, waypoint.y, waypoint.z);

    const bool near_obstacle = obstacleDataFresh(ctx) && obstacleNearSegment(
      ctx, ctx.position_enu.x, ctx.position_enu.y, waypoint.x, waypoint.y);
    if (near_obstacle) {
      RCLCPP_WARN(
        logger_,
        "[SNAKE_EGO] obstacle near segment: aircraft=(%.2f, %.2f), "
        "snake_target=(%.2f, %.2f)",
        ctx.position_enu.x, ctx.position_enu.y, waypoint.x, waypoint.y);
      if (!selectAvoidanceTarget(ctx)) {
        failAndLand(ctx, iface, "no safe snake waypoint for EGO avoidance");
        return Status::RUNNING;
      }
      beginAvoidance(ctx, iface);
      phase_ = Phase::AVOIDING;
      return Status::RUNNING;
    }

    const auto status = snake_.tick(ctx, iface, dt);
    if (status == Status::FAILURE) {
      failAndLand(ctx, iface, "snake task failed");
      return Status::RUNNING;
    }
    if (status == Status::SUCCESS) {
      RCLCPP_INFO(logger_, "[SNAKE_EGO] phase=%s: snake route complete",
                  phaseName());
    }
    return status;
  }




  avoidance_elapsed_s_ += dt;
  SnakeGridTask::WaypointInfo avoidance_target;
  const bool avoidance_target_valid =
    snake_.waypointAt(avoidance_target_index_, avoidance_target);
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "[SNAKE_EGO] phase=%s EGO target=%s(%d, %d) [%zu/%zu], "
    "target=(%.2f, %.2f, %.2f), elapsed=%.1f/%.1f s, "
    "aircraft=(%.2f, %.2f, %.2f)",
    phaseName(), avoidance_target_valid ? "cell=" : "invalid cell=",
    avoidance_target.ix, avoidance_target.iy, avoidance_target_index_ + 1,
    snake_.totalWaypoints(), avoidance_target.x, avoidance_target.y,
    avoidance_target.z, avoidance_elapsed_s_, cfg_.avoidance_timeout_s,
    ctx.position_enu.x, ctx.position_enu.y, ctx.position_enu.z);
  if (avoidance_elapsed_s_ >= cfg_.avoidance_timeout_s) {
    failAndLand(ctx, iface, "EGO avoidance timed out");
    return Status::RUNNING;
  }

  const auto status = ego_.tick(ctx, iface, dt);
  if (status == Status::SUCCESS) {
    // The EGO sub-task has completed this avoidance maneuver.  Close its
    // lifecycle before handing control back to the snake sub-task.
    ego_.onExit(ctx, iface);
    snake_.resumeAfterWaypoint(avoidance_target_index_, ctx);
    phase_ = Phase::SNAKE;
    avoidance_elapsed_s_ = 0.0;
    RCLCPP_INFO(logger_,
                "[SNAKE_EGO] EGO avoidance succeeded; resume phase=%s at "
                "snake waypoint %zu/%zu",
                phaseName(), snake_.currentIndex() + 1,
                snake_.totalWaypoints());
  } else if (status == Status::FAILURE) {
    failAndLand(ctx, iface, "EGO avoidance failed");
  }
  return Status::RUNNING;
}

void SnakeEgoAvoidTask::onExit(Context &ctx, MavrosIface &iface) {
  if (phase_ == Phase::AVOIDING) ego_.onExit(ctx, iface);
  if (phase_ == Phase::LANDING) land_.onExit(ctx, iface);
}

bool SnakeEgoAvoidTask::obstacleDataFresh(const Context &ctx) const {
  if (!ctx.occupancy_grid_valid || ctx.occupancy_grid_resolution <= 0.0 ||
      ctx.occupancy_grid_width == 0 || ctx.occupancy_grid_height == 0) {
    return false;
  }
  const auto now_us = nowUs(clock_);
  if (ctx.occupancy_grid_stamp_us == 0 || now_us < ctx.occupancy_grid_stamp_us) {
    return false;
  }
  const auto age = now_us - ctx.occupancy_grid_stamp_us;
  return
         age <= static_cast<std::uint64_t>(
           std::max(0.0, cfg_.occupancy_timeout_s) * 1e6);
}

bool SnakeEgoAvoidTask::occupiedAt(
  const Context &ctx, double x, double y) const {
  if (!obstacleDataFresh(ctx)) return false;
  const int ix = static_cast<int>(
    std::floor((x - ctx.occupancy_grid_origin_x) /
               ctx.occupancy_grid_resolution));
  const int iy = static_cast<int>(
    std::floor((y - ctx.occupancy_grid_origin_y) /
               ctx.occupancy_grid_resolution));
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(ctx.occupancy_grid_width) ||
      iy >= static_cast<int>(ctx.occupancy_grid_height)) {
    return false;
  }
  const auto index = static_cast<std::size_t>(iy) * ctx.occupancy_grid_width +
                     static_cast<std::size_t>(ix);
  return index < ctx.occupancy_grid_data.size() &&
         ctx.occupancy_grid_data[index] >= cfg_.occupied_threshold;
}


//蛇形预设路径的下一段附近有没有障碍物，如果有，就触发 EGO 避障
//如何判断路径上有无障碍物：对当前蛇形航段的起点和终点，计算出一个矩形区域（包含一定的安全边距），然后遍历这个矩形区域内的所有栅格单元，如果有任何一个栅格单元被占据（即其值大于等于 occupied_threshold），并且该栅格单元到蛇形航段的距离小于等于安全边距，则认为路径上有障碍物，触发 EGO 避障。
bool SnakeEgoAvoidTask::obstacleNearSegment(
  const Context &ctx, double x0, double y0, double x1, double y1) const {
  if (!obstacleDataFresh(ctx)) return false;
  const double resolution = ctx.occupancy_grid_resolution;
  const double margin = std::max(0.0, cfg_.trigger_distance_m) +
                        std::sqrt(2.0) * resolution;
  const double min_x = std::min(x0, x1) - margin;
  const double max_x = std::max(x0, x1) + margin;
  const double min_y = std::min(y0, y1) - margin;
  const double max_y = std::max(y0, y1) + margin;
  const int ix0 = static_cast<int>(std::floor(
    (min_x - ctx.occupancy_grid_origin_x) / resolution));
  const int ix1 = static_cast<int>(std::floor(
    (max_x - ctx.occupancy_grid_origin_x) / resolution));
  const int iy0 = static_cast<int>(std::floor(
    (min_y - ctx.occupancy_grid_origin_y) / resolution));
  const int iy1 = static_cast<int>(std::floor(
    (max_y - ctx.occupancy_grid_origin_y) / resolution));

  for (int iy = std::max(0, iy0);
       iy <= std::min(iy1, static_cast<int>(ctx.occupancy_grid_height) - 1);
       ++iy) {
    for (int ix = std::max(0, ix0);
         ix <= std::min(ix1, static_cast<int>(ctx.occupancy_grid_width) - 1);
         ++ix) {
      const auto index = static_cast<std::size_t>(iy) * ctx.occupancy_grid_width +
                         static_cast<std::size_t>(ix);
      if (index >= ctx.occupancy_grid_data.size() ||
          ctx.occupancy_grid_data[index] < cfg_.occupied_threshold) {
        continue;
      }
      const double cell_x = ctx.occupancy_grid_origin_x +
                            (static_cast<double>(ix) + 0.5) * resolution;
      const double cell_y = ctx.occupancy_grid_origin_y +
                            (static_cast<double>(iy) + 0.5) * resolution;
      if (pointSegmentDistance(cell_x, cell_y, x0, y0, x1, y1) <= margin) {
        return true;
      }
    }
  }
  return false;
}
//从当前蛇形航点开始，向后寻找第一个“不被障碍占用”的航点，并把它设为 EGO 绕障后的目标点。
bool SnakeEgoAvoidTask::selectAvoidanceTarget(const Context &ctx) {
  SnakeGridTask::WaypointInfo waypoint;
  for (std::size_t i = snake_.currentIndex();
       snake_.waypointAt(i, waypoint); ++i) {
    if (!occupiedAt(ctx, waypoint.x, waypoint.y)) {
      avoidance_target_index_ = i;
      return true;
    }
  }
  return false;
}

void SnakeEgoAvoidTask::beginAvoidance(Context &ctx, MavrosIface &iface) {
  SnakeGridTask::WaypointInfo waypoint;
  snake_.waypointAt(avoidance_target_index_, waypoint);
  RCLCPP_INFO(
    logger_, "[SNAKE_EGO] begin EGO avoidance: target cell=(%d, %d) "
    "[%zu/%zu], target=(%.2f, %.2f, %.2f)", waypoint.ix, waypoint.iy,
    avoidance_target_index_ + 1, snake_.totalWaypoints(), waypoint.x,
    waypoint.y, waypoint.z);
  snake_.onPause(ctx, iface);
  ego_.setTargetEnu(waypoint.x, waypoint.y, waypoint.z);
  ego_.onEnter(ctx, iface);
  avoidance_elapsed_s_ = 0.0;
}

ITask::Status SnakeEgoAvoidTask::tickLanding(
  Context &ctx, MavrosIface &iface, double dt_s) {
  const auto status = land_.tick(ctx, iface, dt_s);
  if (status == Status::SUCCESS) {
    ctx.fault = failure_reason_;
    phase_ = Phase::FAILED;
    return Status::FAILURE;
  }
  if (status == Status::FAILURE) {
    ctx.fault = failure_reason_ + "; landing failed";
    phase_ = Phase::FAILED;
    return Status::FAILURE;
  }
  return Status::RUNNING;
}

void SnakeEgoAvoidTask::failAndLand(
  Context &ctx, MavrosIface &iface, const char *reason) {
  if (phase_ == Phase::LANDING || phase_ == Phase::FAILED) return;
  failure_reason_ = reason;
  if (phase_ == Phase::AVOIDING) ego_.onExit(ctx, iface);
  land_.onEnter(ctx, iface);
  phase_ = Phase::LANDING;
  RCLCPP_ERROR(logger_, "[SNAKE_EGO] phase=%s: %s; starting landing",
               phaseName(), failure_reason_.c_str());
}

}  // namespace offboard_core_pkg
