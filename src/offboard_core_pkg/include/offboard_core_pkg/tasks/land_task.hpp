#pragma once

#include <string>

#include "offboard_core_pkg/itask.hpp"

namespace offboard_core_pkg {

class LandTask final : public ITask {
public:
  explicit LandTask(double timeout_s = 15.0, double retry_interval_s = 1.0);

  std::string name() const override;
  void onEnter(Context &ctx, MavrosIface &iface) override;
  Status tick(Context &ctx, MavrosIface &iface, double dt_s) override;

private:
  double timeout_s_;
  double retry_interval_s_;
  double elapsed_s_{0.0};
  double retry_elapsed_s_{0.0};
  double approach_height_m_{0.0};
  double locked_x_m_{0.0};
  double locked_y_m_{0.0};
  bool approach_initialized_{false};
  bool horizontal_position_locked_{false};
  bool request_pending_{false};
  bool request_accepted_{false};
};

}  // namespace offboard_core_pkg
