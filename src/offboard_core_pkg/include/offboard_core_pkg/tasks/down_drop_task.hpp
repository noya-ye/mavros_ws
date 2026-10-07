#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/tasks/goto_task.hpp"

namespace offboard_core_pkg {
namespace utils { class SendSerial; }

class DownDropTask final : public ITask {
public:
  struct obj_id {
    int id{0};
    double offset_x{0.0};
    double offset_y{0.0};
  };

  DownDropTask(
    rclcpp::Logger logger, double land_height, obj_id target,
    std::string serial_device, unsigned int baud_rate = 115200);
  ~DownDropTask() override;

  std::string name() const override;
  void setTarget(obj_id target);
  void onEnter(Context &ctx, MavrosIface &iface) override;
  Status tick(Context &ctx, MavrosIface &iface, double dt_s) override;
  void onExit(Context &ctx, MavrosIface &iface) override;

private:
  enum class Phase { DOWN, MOVE, DROP, FAILED, FINISHED };

  const char *phaseName() const;
  void fail(Context &ctx, const std::string &reason);

  rclcpp::Logger logger_;
  double land_height_;
  obj_id target_;
  std::unique_ptr<utils::SendSerial> serial_;
  Phase phase_{Phase::FAILED};
  GotoTask goto_{0.0, 0.0, 0.0, 0.1};
  bool valid_config_{false};
};

}  // namespace offboard_core_pkg
