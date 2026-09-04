#pragma once

#include <string>

#include "offboard_core_pkg/itask.hpp"

namespace offboard_core_pkg {

class HoverTask final : public ITask {
public:
  explicit HoverTask(double duration_s);

  std::string name() const override;
  void onEnter(Context &ctx, MavrosIface &iface) override;
  Status tick(Context &ctx, MavrosIface &iface, double dt_s) override;

private:
  double duration_s_;
  double elapsed_s_{0.0};
};

}  // namespace offboard_core_pkg
