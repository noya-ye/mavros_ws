#pragma once

#include <chrono>
#include <cstdint>

#include "offboard_core_pkg/itask.hpp"

namespace offboard_core_pkg {

class RedCrossAlignTask final : public ITask {
public:
  RedCrossAlignTask(
      double pixels_per_meter,
      int stable_frames,
      double arrive_distance_m,
      double max_step_m);

  std::string name() const override;
  void onEnter(Context &ctx, MavrosIface &iface) override;
  Status tick(Context &ctx, MavrosIface &iface, double dt_s) override;
  void onPause(Context &ctx, MavrosIface &iface) override;
  void onResume(Context &ctx, MavrosIface &iface) override;

private:
  void holdPosition(Context &ctx) const;

  double pixels_per_meter_{0.0};
  int stable_frames_{0};
  double arrive_distance_m_{0.0};
  double max_step_m_{0.0};
  bool valid_config_{false};
  double align_height_{0.0};
  int stable_count_{0};
  std::uint64_t last_seq_{0};
  bool detection_was_available_{false};
  std::chrono::steady_clock::time_point last_status_log_{};
};

}  // namespace offboard_core_pkg
