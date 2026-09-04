#pragma once

#include <string>

#include "offboard_core_pkg/itask.hpp"

namespace offboard_core_pkg {

class PresetpointTask final : public ITask {
public:
  explicit PresetpointTask(double duration_s);

  std::string name() const override;
  void onEnter(Context &ctx, MavrosIface &iface) override;
  Status tick(Context &ctx, MavrosIface &iface, double dt_s) override;

private:
  double duration_s_;
  double elapsed_s_{0.0};
  bool target_initialized_{false};
};

}  // namespace offboard_core_pkg
