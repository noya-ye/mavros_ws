#pragma once

#include <string>

#include "offboard_core_pkg/itask.hpp"

namespace offboard_core_pkg {

class TakeoffTask final : public ITask {
public:
  TakeoffTask(double height_m, double tolerance_m, double timeout_s = 30.0);

  std::string name() const override;
  void onEnter(Context &ctx, MavrosIface &iface) override;
  Status tick(Context &ctx, MavrosIface &iface, double dt_s) override;

private:
  double height_m_;
  double tolerance_m_;
  double timeout_s_;
  double elapsed_s_{0.0};
  bool target_initialized_{false};
};

}  // namespace offboard_core_pkg
