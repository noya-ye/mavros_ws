#include "offboard_core_pkg/tasks/ego_goto_task.hpp"

#include <algorithm>
#include <cmath>

namespace offboard_core_pkg {

// ============================================================================
// 设置绝对 ENU 目标
// ============================================================================
void EgoGotoTask::setTargetEnu(double x, double y, double z) {
  target_x_ = x;
  target_y_ = y;
  target_z_ = z;

  target_override_valid_ =
      std::isfinite(x) &&
      std::isfinite(y) &&
      std::isfinite(z);
}


// ============================================================================
// Task 进入
// ============================================================================
void EgoGotoTask::onEnter(Context &ctx, MavrosIface &) {
  started_ = false;
  elapsed_ = 0.0;
  stable_ = 0.0;
  held_altitude_valid_ = ctx.position_valid && ctx.finitePosition();
  if (held_altitude_valid_) {
    held_altitude_enu_ = ctx.position_enu.z;
  }

  planner_.reset(ctx);

  // 清除上一轮 EGO 控制指令
  ctx.ego_cmd_valid = false;

  // 在 EGO 真正产生轨迹之前，默认采用位置模式
  ctx.setpoint_mode = SetpointMode::POSITION;

  // --------------------------------------------------------------------------
  // 如果没有通过 setTargetEnu() 设置绝对目标，
  // 则使用配置文件里的相对目标。
  // --------------------------------------------------------------------------
  if (!target_override_valid_) {
    if (ctx.home_initialized) {
      target_x_ = ctx.home_enu.x + cfg_.x_rel;
      target_y_ = ctx.home_enu.y + cfg_.y_rel;
      target_z_ = ctx.home_enu.z + cfg_.height_m;
    } else {
      target_x_ = ctx.position_enu.x + cfg_.x_rel;
      target_y_ = ctx.position_enu.y + cfg_.y_rel;
      target_z_ = ctx.position_enu.z + cfg_.height_m;
    }
  }

  // EGO is a 2D planner.  Lock the MAVROS height at the instant control is
  // handed to EGO, regardless of its nominal goal or PositionCommand Z.
  if (held_altitude_valid_) {
    target_z_ = held_altitude_enu_;
  }

  // --------------------------------------------------------------------------
  // EGO odom 已经准备好：
  // 只在这里发布一次目标。
  //
  // 重要：
  // 不再周期性重新计算并发布目标，否则 MAVROS position 和 EGO odom
  // 的厘米级噪声会导致 EGO 不断收到“新目标”。
  // --------------------------------------------------------------------------
  if (ctx.ego_odom_valid) {
    publishGoal(ctx);
    started_ = true;
  }
}


// ============================================================================
// Task 主循环
// ============================================================================
ITask::Status EgoGotoTask::tick(
    Context &ctx,
    MavrosIface &,
    double dt_s)
{
  // --------------------------------------------------------------------------
  // 1. MAVROS 位置信息无效
  // --------------------------------------------------------------------------
  if (!ctx.position_valid || !ctx.finitePosition()) {
    hold(ctx);
    stable_ = 0.0;
    return Status::RUNNING;
  }

  if (!held_altitude_valid_) {
    held_altitude_enu_ = ctx.position_enu.z;
    held_altitude_valid_ = true;
    target_z_ = held_altitude_enu_;
  }

  const double dt =
      std::clamp(
          std::isfinite(dt_s) ? dt_s : 0.0,
          0.0,
          0.2);

  // --------------------------------------------------------------------------
  // 2. 等待 EGO odom
  //
  // 如果 onEnter() 时 EGO odom 尚未准备好，
  // 在这里等待，并在第一次有效时发布一次 Goal。
  // --------------------------------------------------------------------------
  if (!started_) {
    if (!ctx.ego_odom_valid) {
      hold(ctx);
      stable_ = 0.0;
      return Status::RUNNING;
    }

    publishGoal(ctx);
    started_ = true;
  }

  // ==========================================================================
  // 3. 先判断是否已经到达最终目标
  //
  // 必须在 planner_.plan() 前面。
  //
  // 原来的逻辑：
  //
  //   planner_.plan()
  //        ↓
  //   路径很短
  //        ↓
  //   B-spline control points 不足
  //        ↓
  //   return RUNNING
  //        ↓
  //   永远执行不到到达判断
  //
  // 现在：
  //
  //   先判断是否已经进入目标范围
  //        ↓
  //   是 → 不再调用 EGO
  //        ↓
  //   直接保持最终目标
  // ==========================================================================
  const double ex =
      target_x_ - ctx.position_enu.x;

  const double ey =
      target_y_ - ctx.position_enu.y;

  const double ez =
      target_z_ - ctx.position_enu.z;

  const double error_xy =
      std::hypot(ex, ey);

  const double error_z =
      std::fabs(ez);

  const double vxy =
      std::hypot(
          ctx.velocity_enu.x,
          ctx.velocity_enu.y);

  const double vz =
      std::fabs(ctx.velocity_enu.z);

  const bool position_arrived =
      error_xy <= cfg_.arrive_xy_m &&
      error_z <= cfg_.arrive_z_m;

  const bool velocity_stable =
      vxy <= cfg_.stable_vxy_mps &&
      vz <= cfg_.stable_vz_mps;


  // ==========================================================================
  // 4. 已经进入目标位置范围
  // ==========================================================================
  if (position_arrived) {

    // ------------------------------------------------------------------------
    // 不再让 EGO 对几十厘米甚至几厘米的路径继续重新规划。
    //
    // 这里直接让 MAVROS 保持最终目标位置。
    // ------------------------------------------------------------------------
    ctx.position_setpoint_enu = {
        target_x_,
        target_y_,
        target_z_
    };

    ctx.velocity_setpoint_enu = {
        0.0,
        0.0,
        0.0
    };

    ctx.acceleration_setpoint_enu = {
        0.0,
        0.0,
        0.0
    };

    ctx.yaw_setpoint_enu =
        cfg_.yaw_local;

    ctx.setpoint_mode =
        SetpointMode::POSITION;

    ctx.use_position_velocity_acceleration =
        false;

    ctx.publish_position_setpoint =
        true;


    // ------------------------------------------------------------------------
    // 位置已经满足要求之后，再检查速度是否稳定。
    //
    // 只有：
    //
    //   位置满足
    //   +
    //   水平速度满足
    //   +
    //   垂直速度满足
    //
    // 才累计 stable 时间。
    // ------------------------------------------------------------------------
    if (velocity_stable) {
      stable_ += dt;
    } else {
      stable_ = 0.0;
    }


    // ------------------------------------------------------------------------
    // 连续稳定足够时间 → Task 成功
    // ------------------------------------------------------------------------
    if (stable_ >= cfg_.stable_required_s) {
      hold(ctx);
      return Status::SUCCESS;
    }

    return Status::RUNNING;
  }


  // ==========================================================================
  // 5. 尚未进入目标区域
  //
  // 此时才需要 EGO 继续进行避障规划。
  // ==========================================================================
  stable_ = 0.0;

  EgoVelPlanner::Debug dbg;

  const auto result =
      planner_.plan(ctx, &dbg);

  // Preserve the entry altitude even when EGO input becomes stale and the
  // planner falls back to its own hold behavior.
  ctx.position_setpoint_enu.z = held_altitude_enu_;
  ctx.velocity_setpoint_enu.z = 0.0;
  ctx.acceleration_setpoint_enu.z = 0.0;

  if (result != EgoVelPlanner::Result::OK) {
    return Status::RUNNING;
  }

  return Status::RUNNING;
}


// ============================================================================
// Task 退出
// ============================================================================
void EgoGotoTask::onExit(
    Context &ctx,
    MavrosIface &)
{
  hold(ctx);
}


// ============================================================================
// 向 EGO 发布目标
// ============================================================================
void EgoGotoTask::publishGoal(
    const Context &ctx)
{
  if (!goal_pub_ || !ctx.ego_odom_valid) {
    return;
  }

  // --------------------------------------------------------------------------
  // 计算：
  //
  // MAVROS ENU 当前点 → MAVROS ENU 最终目标
  //
  // 的相对位移。
  // --------------------------------------------------------------------------
  const double dx_enu =
      target_x_ - ctx.position_enu.x;

  const double dy_enu =
      target_y_ - ctx.position_enu.y;


  // --------------------------------------------------------------------------
  // ENU 相对位移
  // →
  // EGO/Lidar 坐标系相对位移
  // --------------------------------------------------------------------------
  double ego_dx = 0.0;
  double ego_dy = 0.0;

  inverseMap(
      dx_enu,
      dy_enu,
      ego_dx,
      ego_dy);


  // --------------------------------------------------------------------------
  // 以当前 EGO odom 为基准构造 EGO 绝对目标。
  //
  // 注意：
  // 此函数现在在整个 EgoGotoTask 中只应该调用一次。
  //
  // 不能每秒重新调用，否则：
  //
  //   MAVROS position noise
  //   +
  //   EGO odom noise
  //
  // 会导致最终 goal 每次漂移几厘米。
  // --------------------------------------------------------------------------
  target_ego_x_ =
      ctx.ego_odom_position.x + ego_dx;

  target_ego_y_ =
      ctx.ego_odom_position.y + ego_dy;

  target_ego_z_ =
      ctx.ego_odom_position.z +
      (target_z_ - ctx.position_enu.z);


  // --------------------------------------------------------------------------
  // 发布 PoseStamped Goal
  // --------------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped msg;

  msg.header.stamp =
      clock_->now();

  msg.header.frame_id =
      cfg_.goal_frame;

  msg.pose.position.x =
      target_ego_x_;

  msg.pose.position.y =
      target_ego_y_;

  msg.pose.position.z =
      target_ego_z_;

  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  goal_pub_->publish(msg);
}


// ============================================================================
// MAVROS ENU 相对位移
// →
// EGO/Lidar 相对位移
// ============================================================================
void EgoGotoTask::inverseMap(
    double x,
    double y,
    double &ox,
    double &oy) const
{
  // --------------------------------------------------------------------------
  // 防止 x_sign / y_sign 被错误设置成 0
  // --------------------------------------------------------------------------
  const double sx =
      std::fabs(cfg_.planner.x_sign) > 1e-6
          ? cfg_.planner.x_sign
          : 1.0;

  const double sy =
      std::fabs(cfg_.planner.y_sign) > 1e-6
          ? cfg_.planner.y_sign
          : 1.0;


  // --------------------------------------------------------------------------
  // 恢复符号映射
  // --------------------------------------------------------------------------
  const double rx =
      x / sx;

  const double ry =
      y / sy;


  // --------------------------------------------------------------------------
  // 逆 yaw 对齐
  // --------------------------------------------------------------------------
  const double c =
      std::cos(cfg_.planner.yaw_align_rad);

  const double s =
      std::sin(cfg_.planner.yaw_align_rad);

  const double mx =
      c * rx + s * ry;

  const double my =
      -s * rx + c * ry;


  // --------------------------------------------------------------------------
  // 处理 XY swap
  // --------------------------------------------------------------------------
  if (cfg_.planner.swap_xy) {
    ox = my;
    oy = mx;
  } else {
    ox = mx;
    oy = my;
  }
}


// ============================================================================
// 保持当前位置
// ============================================================================
void EgoGotoTask::hold(
    Context &ctx)
{
  ctx.position_setpoint_enu =
      ctx.position_enu;
  if (held_altitude_valid_) {
    ctx.position_setpoint_enu.z = held_altitude_enu_;
  }

  ctx.velocity_setpoint_enu = {
      0.0,
      0.0,
      0.0
  };

  ctx.acceleration_setpoint_enu = {
      0.0,
      0.0,
      0.0
  };

  ctx.yaw_setpoint_enu =
      cfg_.yaw_local;

  ctx.setpoint_mode =
      SetpointMode::POSITION;

  ctx.use_position_velocity_acceleration =
      false;

  ctx.publish_position_setpoint =
      true;
}

}  // namespace offboard_core_pkg
