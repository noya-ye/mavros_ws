#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "offboard_core_pkg/itask.hpp"

namespace offboard_core_pkg {

class AlignDownTask final : public ITask {
public:
  AlignDownTask(
      double pixels_per_meter,
      int stable_frames,
      double arrive_distance_m,
      double max_step_m);

  std::string name() const override;

  void onEnter(Context &ctx, MavrosIface &iface) override;

  Status tick(
      Context &ctx,
      MavrosIface &iface,
      double dt) override;

  void onPause(Context &ctx, MavrosIface &iface) override;

  void onResume(Context &ctx, MavrosIface &iface) override;

  bool reached_arrival_tolerance() const {
    return reached_arrival_tolerance_;
  }

  bool target_confirmed() const {
    return target_check_done_ && !target_skipped_;
  }

private:
  void holdPosition(Context &ctx) const;

private:
  // 图像比例：多少像素对应 1 米
  double pixels_per_meter_{0.0};

  // 连续多少帧满足误差要求才算完成
  int stable_frames_{0};

  // 到达判定距离
  double arrive_distance_m_{0.0};

  // 单次视觉纠偏允许的最大水平步长
  double max_step_m_{0.10};

  // 参数是否合法
  bool valid_config_{false};

  // 进入任务时的固定高度
  double align_height_{0.0};

  // 当前连续稳定帧数
  int stable_count_{0};

  // 用于判断是否收到新的视觉帧
  uint64_t last_circle_seq_{0};
  uint64_t last_contour_seq_{0};

  // 上一次使用的检测源
  bool last_was_circle_{false};

  // 是否曾经有有效检测
  bool detection_was_available_{false};

  // 首次进入到达阈值时是否已经完成 YOLO 目标确认
  bool target_check_done_{false};
  bool target_skipped_{false};
  bool reached_arrival_tolerance_{false};

  // 连续低置信度 YOLO 帧计数，以及最近一次已检查的帧序号
  int low_confidence_yolo_frames_{0};
  uint64_t last_yolo_check_seq_{0};

  // 日志限频
  std::chrono::steady_clock::time_point last_status_log_{};
};

}  // namespace offboard_core_pkg
