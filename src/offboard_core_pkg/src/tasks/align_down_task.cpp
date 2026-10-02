#include "offboard_core_pkg/tasks/align_down_task.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "offboard_core_pkg/context.hpp"
#include "rclcpp/rclcpp.hpp"

namespace offboard_core_pkg {

namespace {

constexpr auto kDetectionTimeout = std::chrono::milliseconds(500);

rclcpp::Logger logger() {
  return rclcpp::get_logger("align_down");
}

}  // namespace


AlignDownTask::AlignDownTask(
    double pixels_per_meter,
    int stable_frames,
    double arrive_distance_m,
    double max_step_m)
    : pixels_per_meter_(pixels_per_meter),
      stable_frames_(stable_frames),
      arrive_distance_m_(arrive_distance_m),
      max_step_m_(max_step_m),
      valid_config_(
          std::isfinite(pixels_per_meter) &&
          pixels_per_meter > 0.0 &&
          stable_frames > 0 &&
          std::isfinite(arrive_distance_m) &&
          arrive_distance_m > 0.0 &&
          std::isfinite(max_step_m) &&
          max_step_m > 0.0) {}


std::string AlignDownTask::name() const {
  return "align_down";
}


void AlignDownTask::holdPosition(Context &ctx) const {
  if (!ctx.position_valid || !ctx.finitePosition()) {
    return;
  }

  // 当前 XY 悬停
  // Z 始终保持进入 AlignDown 时的高度
  ctx.position_setpoint_enu = {
      ctx.position_enu.x,
      ctx.position_enu.y,
      align_height_
  };

  // Yaw 始终锁死为 Home yaw
  if (std::isfinite(ctx.home_yaw_enu)) {
    ctx.yaw_setpoint_enu = ctx.home_yaw_enu;
  }

  ctx.setpoint_mode = SetpointMode::POSITION;
  ctx.use_position_velocity_acceleration = false;
  ctx.publish_position_setpoint = true;
}


void AlignDownTask::onEnter(
    Context &ctx,
    MavrosIface &) {

  stable_count_ = 0;

  // 记录进入任务时的高度
  align_height_ = ctx.position_enu.z;

  last_circle_seq_ = ctx.down_circle_seq;
  last_contour_seq_ = ctx.down_contour_seq;

  last_was_circle_ = false;

  ctx.fault.clear();

  detection_was_available_ = false;
  last_status_log_ = {};

  if (!valid_config_) {
    ctx.fault = "align_down parameters are invalid";

    RCLCPP_ERROR(
        logger(),
        "[ALIGN_DOWN] invalid config: "
        "pixels_per_meter=%.2f "
        "stable_frames=%d "
        "arrive_distance=%.3f m "
        "max_step=%.3f m",
        pixels_per_meter_,
        stable_frames_,
        arrive_distance_m_,
        max_step_m_);

    return;
  }

  if (!std::isfinite(ctx.home_yaw_enu)) {
    ctx.fault = "align_down home yaw is invalid";

    RCLCPP_ERROR(
        logger(),
        "[ALIGN_DOWN] home_yaw_enu is invalid");

    return;
  }

  // 一进入任务就立即悬停，
  // 同时把 yaw 拉到并锁定在 home_yaw_enu
  holdPosition(ctx);

  RCLCPP_INFO(
      logger(),
      "[ALIGN_DOWN] started: "
      "pixels_per_meter=%.2f "
      "stable_frames=%d "
      "arrive_distance=%.3f m "
      "max_step=%.3f m "
      "hold_height=%.2f m "
      "locked_yaw=%.3f rad",
      pixels_per_meter_,
      stable_frames_,
      arrive_distance_m_,
      max_step_m_,
      align_height_,
      ctx.home_yaw_enu);
}


ITask::Status AlignDownTask::tick(
    Context &ctx,
    MavrosIface &,
    double) {

  if (!valid_config_) {
    return Status::FAILURE;
  }

  /*
   * ---------------------------------------------------------
   * 1. 检查飞行器状态
   * ---------------------------------------------------------
   */

  if (!ctx.connected ||
      !ctx.position_valid ||
      !ctx.finitePosition() ||
      !std::isfinite(ctx.yaw_enu) ||
      !std::isfinite(ctx.home_yaw_enu)) {

    if (detection_was_available_) {
      RCLCPP_WARN(
          logger(),
          "[ALIGN_DOWN] vehicle state became invalid; "
          "resetting stability");
    }

    detection_was_available_ = false;
    stable_count_ = 0;

    return Status::RUNNING;
  }

  const auto now = std::chrono::steady_clock::now();

  const auto fresh =
      [now](const std::chrono::steady_clock::time_point &stamp) {

        return
            stamp != std::chrono::steady_clock::time_point{} &&
            stamp <= now &&
            now - stamp <= kDetectionTimeout;
      };


  /*
   * ---------------------------------------------------------
   * 2. 判断 circle / contour 是否有效
   * ---------------------------------------------------------
   */

  const bool circle =
      ctx.down_circle_seq != 0 &&
      fresh(ctx.down_circle_stamp);

  const bool contour =
      ctx.down_contour_seq != 0 &&
      fresh(ctx.down_contour_stamp);


  /*
   * circle 优先。
   *
   * circle 不可用时才使用 contour。
   */

  if (!circle && !contour) {

    if (detection_was_available_) {
      RCLCPP_WARN(
          logger(),
          "[ALIGN_DOWN] down-facing detection timed out; "
          "holding position");
    }

    detection_was_available_ = false;
    stable_count_ = 0;

    holdPosition(ctx);

    return Status::RUNNING;
  }


  if (!detection_was_available_) {
    RCLCPP_INFO(
        logger(),
        "[ALIGN_DOWN] fresh %s detection available",
        circle ? "circle" : "contour");
  }

  detection_was_available_ = true;


  /*
   * ---------------------------------------------------------
   * 3. 选择当前检测源
   * ---------------------------------------------------------
   */

  const auto &offset =
      circle
          ? ctx.down_circle_offset_px
          : ctx.down_contour_offset_px;

  const auto seq =
      circle
          ? ctx.down_circle_seq
          : ctx.down_contour_seq;

  auto &last_seq =
      circle
          ? last_circle_seq_
          : last_contour_seq_;


  /*
   * 同一视觉帧只处理一次。
   */

  if (seq == last_seq) {
    return Status::RUNNING;
  }

  last_seq = seq;


  /*
   * circle <-> contour 切换时，
   * 稳定计数清零。
   */

  if (circle != last_was_circle_) {

    stable_count_ = 0;

    RCLCPP_INFO(
        logger(),
        "[ALIGN_DOWN] switching detection source to %s",
        circle ? "circle" : "contour");
  }

  last_was_circle_ = circle;


  /*
   * ---------------------------------------------------------
   * 4. 检查视觉数据
   * ---------------------------------------------------------
   */

  if (!std::isfinite(offset.x) ||
      !std::isfinite(offset.y)) {

    RCLCPP_WARN(
        logger(),
        "[ALIGN_DOWN] received non-finite %s offset; "
        "holding position",
        circle ? "circle" : "contour");

    stable_count_ = 0;

    holdPosition(ctx);

    return Status::RUNNING;
  }


  /*
   * ---------------------------------------------------------
   * 5. 像素误差 -> 机体系米制误差
   * ---------------------------------------------------------
   *
   * 当前约定：
   *
   * offset.x
   *     ↓
   * body forward
   *
   * offset.y
   *     ↓
   * body left
   *
   * 即 ROS FLU：
   *
   * x = forward
   * y = left
   */

  const double forward =
      offset.x / pixels_per_meter_;

  const double left =
      offset.y / pixels_per_meter_;

  const double distance =
      std::hypot(forward, left);


  /*
   * ---------------------------------------------------------
   * 6. 已经进入允许误差范围
   * ---------------------------------------------------------
   */

  if (distance <= arrive_distance_m_) {

    // 位置停止移动
    // yaw 仍然持续锁定 home_yaw
    holdPosition(ctx);

    ++stable_count_;

    const auto log_now =
        std::chrono::steady_clock::now();

    if (last_status_log_ ==
            std::chrono::steady_clock::time_point{} ||
        log_now - last_status_log_ >=
            std::chrono::seconds(1)) {

      RCLCPP_INFO(
          logger(),
          "[ALIGN_DOWN] within tolerance: "
          "source=%s "
          "offset=(%.1f, %.1f) px "
          "distance=%.3f m "
          "stable=%d/%d",
          circle ? "circle" : "contour",
          offset.x,
          offset.y,
          distance,
          stable_count_,
          stable_frames_);

      last_status_log_ = log_now;
    }


    /*
     * 连续 stable_frames 帧满足要求
     * 才真正完成任务。
     */

    if (stable_count_ >= stable_frames_) {

      RCLCPP_INFO(
          logger(),
          "[ALIGN_DOWN] alignment complete: "
          "distance=%.3f m "
          "stable_frames=%d",
          distance,
          stable_count_);

      return Status::SUCCESS;
    }

    return Status::RUNNING;
  }


  /*
   * ---------------------------------------------------------
   * 7. 尚未到达，开始进行位置纠偏
   * ---------------------------------------------------------
   */

  stable_count_ = 0;


  /*
   * ---------------------------------------------------------
   * 8. 最大单帧步长限制
   * ---------------------------------------------------------
   *
   * 原始：
   *
   *      [forward, left]
   *
   * 假设视觉算出来目标距离飞机 0.8 m，
   * max_step_m = 0.10 m
   *
   * 则：
   *
   *      scale = 0.10 / 0.80
   *
   * 保持方向不变，
   * 但是本次只走 0.10 m。
   */

  const double step_scale =
      std::min(
          1.0,
          max_step_m_ / distance);


  const double step_forward =
      forward * step_scale;

  const double step_left =
      left * step_scale;

  const double actual_step =
      std::hypot(
          step_forward,
          step_left);


  /*
   * ---------------------------------------------------------
   * 9. 机体系误差 -> ENU 世界坐标
   * ---------------------------------------------------------
   *
   * 注意：
   *
   * 这里必须使用实时 ctx.yaw_enu，
   * 而不是 home_yaw_enu。
   *
   * 原因：
   *
   * 相机测得的 forward/left 是在
   * 飞机真实当前机体系下产生的。
   *
   * 即使我们命令飞机保持 home_yaw，
   * 实际 yaw 仍然可能有 1~2° 小误差。
   *
   * 用实际 yaw 转换才能保证方向正确。
   */

  const double c =
      std::cos(ctx.yaw_enu);

  const double s =
      std::sin(ctx.yaw_enu);


  /*
   * Body FL:
   *
   * [forward]
   * [left   ]
   *
   *          ↓
   *
   * ENU:
   *
   * dx = cos(yaw)*forward - sin(yaw)*left
   * dy = sin(yaw)*forward + cos(yaw)*left
   */

  const double step_x_enu =
      c * step_forward -
      s * step_left;

  const double step_y_enu =
      s * step_forward +
      c * step_left;


  /*
   * ---------------------------------------------------------
   * 10. 生成新的 Position Setpoint
   * ---------------------------------------------------------
   *
   * XY：
   *      当前实际位置
   *      +
   *      限幅后的视觉纠偏量
   *
   * Z：
   *      始终使用进入任务时的高度
   */

  ctx.position_setpoint_enu = {
      ctx.position_enu.x + step_x_enu,
      ctx.position_enu.y + step_y_enu,
      align_height_
  };


  /*
   * ---------------------------------------------------------
   * 11. Yaw 锁死为 Home yaw
   * ---------------------------------------------------------
   *
   * 每次 tick 都明确写入，
   * 因此整个 AlignDown 期间
   * yaw setpoint 不随视觉变化。
   */

  ctx.yaw_setpoint_enu =
      ctx.home_yaw_enu;


  ctx.setpoint_mode =
      SetpointMode::POSITION;

  ctx.use_position_velocity_acceleration =
      false;

  ctx.publish_position_setpoint =
      true;


  /*
   * ---------------------------------------------------------
   * 12. 状态日志
   * ---------------------------------------------------------
   */

  const auto log_now =
      std::chrono::steady_clock::now();

  if (last_status_log_ ==
          std::chrono::steady_clock::time_point{} ||
      log_now - last_status_log_ >=
          std::chrono::seconds(1)) {

    RCLCPP_INFO(
        logger(),
        "[ALIGN_DOWN] correcting: "
        "source=%s "
        "offset=(%.1f, %.1f) px "
        "error_body=(%.3f, %.3f) m "
        "distance=%.3f m "
        "step_body=(%.3f, %.3f) m "
        "step=%.3f m "
        "target=(%.2f, %.2f, %.2f) "
        "yaw=%.3f -> locked=%.3f",
        circle ? "circle" : "contour",
        offset.x,
        offset.y,
        forward,
        left,
        distance,
        step_forward,
        step_left,
        actual_step,
        ctx.position_setpoint_enu.x,
        ctx.position_setpoint_enu.y,
        ctx.position_setpoint_enu.z,
        ctx.yaw_enu,
        ctx.home_yaw_enu);

    last_status_log_ = log_now;
  }


  return Status::RUNNING;
}


void AlignDownTask::onPause(
    Context &ctx,
    MavrosIface &) {

  RCLCPP_INFO(
      logger(),
      "[ALIGN_DOWN] paused; "
      "holding current position with home yaw");

  stable_count_ = 0;

  detection_was_available_ = false;

  holdPosition(ctx);
}


void AlignDownTask::onResume(
    Context &ctx,
    MavrosIface &) {

  RCLCPP_INFO(
      logger(),
      "[ALIGN_DOWN] resumed; "
      "waiting for a new detection frame");

  stable_count_ = 0;

  detection_was_available_ = false;

  last_circle_seq_ =
      ctx.down_circle_seq;

  last_contour_seq_ =
      ctx.down_contour_seq;

  holdPosition(ctx);
}

}  // namespace offboard_core_pkg