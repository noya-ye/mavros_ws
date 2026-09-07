#pragma once

#include <string>

#include "offboard_core_pkg/itask.hpp"

namespace offboard_core_pkg {

class GotoTask final : public ITask {
public:
  explicit GotoTask(double x, double y, double z,double tolerance_m );

  std::string name() const override;
  void onEnter(Context &ctx, MavrosIface &iface) override;
  Status tick(Context &ctx, MavrosIface &iface, double dt_s) override;

private:
  double x_, y_, z_;
  double tolerance_m_ = 0.1;
  bool valid_target_{true};
};

}  // namespace offboard_core_pkg
