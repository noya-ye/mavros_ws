#include <chrono>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"

namespace offboard_core_pkg {

class SnakeNode final : public rclcpp::Node {
public:
  SnakeNode() : Node("snake_node"), iface_(*this, ctx_) {
    const double rate_hz = declare_parameter<double>("setpoint_rate_hz", 20.0);
    const double presetpoint_s = declare_parameter<double>("presetpoint_duration_s", 2.0);
    const double command_timeout_s = declare_parameter<double>("command_timeout_s", 10.0);
    const double command_retry_s = declare_parameter<double>("command_retry_interval_s", 1.0);
    const double takeoff_tolerance_m = declare_parameter<double>("takeoff_tolerance_m", 0.12);
    const double takeoff_timeout_s = declare_parameter<double>("takeoff_timeout_s", 30.0);
    const double endpoint_hover_s = declare_parameter<double>("endpoint_hover_s", 0.8);
    const double land_timeout_s = declare_parameter<double>("land_timeout_s", 15.0);
    const bool auto_start = declare_parameter<bool>("auto_start", true);
    if (rate_hz < 2.0) throw std::invalid_argument("setpoint_rate_hz must be at least 2 Hz");

    SnakeGridTask::Config snake_cfg;
    snake_cfg.first_axis = SnakeGridTask::FirstAxis::X_FIRST;
    snake_cfg.stop_mode = SnakeGridTask::StopMode::LINE_END_ONLY;
    snake_cfg.x_cells = 3;
    snake_cfg.y_cells = 3;
    snake_cfg.include_start_cell = true;
    snake_cfg.hover_s = endpoint_hover_s;

    scheduler_.add(std::make_unique<PresetpointTask>(presetpoint_s));
    scheduler_.add(std::make_unique<SetOffboardTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<ArmTask>(command_timeout_s, command_retry_s));
    scheduler_.add(std::make_unique<TakeoffTask>(0.8, takeoff_tolerance_m, takeoff_timeout_s));
    scheduler_.add(std::make_unique<SnakeGridTask>(get_logger(), snake_cfg));
    scheduler_.add(std::make_unique<LandTask>(land_timeout_s, command_retry_s));
    scheduler_.reset();

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this, auto_start] {
        const auto current = now();
        const double dt_s = (current - last_tick_).seconds();
        last_tick_ = current;
        if (auto_start && !scheduler_.done() && !scheduler_.failed()) {
          scheduler_.tick(ctx_, iface_, dt_s);
        }
        iface_.publishSetpoint();
      });
  }

private:
  Context ctx_;
  MavrosIface iface_;
  Scheduler scheduler_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
};

}  // namespace offboard_core_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_core_pkg::SnakeNode>());
  rclcpp::shutdown();
  return 0;
}
