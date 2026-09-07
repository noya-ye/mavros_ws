#pragma once

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/itask.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace offboard_core_pkg {

class SnakeGridTask final : public ITask {
public:
  enum class FirstAxis { X_FIRST, Y_FIRST };
  enum class StopMode { EVERY_CELL, LINE_END_ONLY };

  struct ObstacleCell { int ix{0}; int iy{0}; };
  struct WaypointInfo {
    int ix{0};
    int iy{0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct Config {
    FirstAxis first_axis{FirstAxis::X_FIRST};
    StopMode stop_mode{StopMode::EVERY_CELL};
    int x_cells{1};
    int y_cells{1};
    double cell_size{0.8};
    int x_sign{1};
    int y_sign{1};
    bool include_start_cell{true};
    double hover_s{0.8};
    double max_step_m{0.03};
    double arrive_xy_m{0.12};
    double arrive_z_m{0.15};
    std::vector<ObstacleCell> obstacle_cells;
  };

  SnakeGridTask(rclcpp::Logger logger, const Config &cfg);

  std::string name() const override;
  void onEnter(Context &, MavrosIface &) override;
  Status tick(Context &, MavrosIface &, double dt_s) override;
  void onExit(Context &, MavrosIface &) override;
  void onPause(Context &, MavrosIface &) override;
  void onResume(Context &, MavrosIface &) override;

  std::uint32_t planId() const;
  bool planReady() const;
  const std::vector<std::string> &routeCells() const;
  std::size_t currentIndex() const;
  std::size_t totalWaypoints() const;
  std::string currentCell() const;
  bool finished() const;
  bool failed() const;
  bool waypointAt(std::size_t index, WaypointInfo &out) const;
  void skipCurrentWaypoint();
  void resumeAfterWaypoint(std::size_t completed_index, Context &ctx);

private:
  struct Waypoint { int ix{0}; int iy{0}; double x{0}; double y{0}; double z{0}; bool hover_after{false}; };
  enum class Phase { MOVING, HOVERING, FINISHED, FAILED };

  void buildWaypoints();
  void buildXFirstWaypoints();
  void buildYFirstWaypoints();
  void pushWaypoint(int ix, int iy, bool line_end);
  Waypoint makeWaypoint(int ix, int iy, bool hover_after) const;
  void rebuildRouteCells();
  std::string cellToName(int ix, int iy) const;
  void moveCommandToward(const Waypoint &, double dt_s);
  bool arrived(const Context &, const Waypoint &) const;
  void publishSetpoint(Context &);
  void nextWaypoint();
  std::size_t displayWaypointIndex() const;

  rclcpp::Logger logger_;
  Config cfg_;
  std::vector<Waypoint> waypoints_;
  std::vector<std::string> route_cells_;
  std::size_t index_{0};
  double origin_x_{0}, origin_y_{0}, target_z_{0};
  double cmd_x_{0}, cmd_y_{0}, cmd_z_{0}, yaw_hold_{0}, hover_elapsed_s_{0};
  Phase phase_{Phase::FINISHED};
  std::uint32_t plan_id_{0};
  bool plan_ready_{false};
};

}  // namespace offboard_core_pkg
