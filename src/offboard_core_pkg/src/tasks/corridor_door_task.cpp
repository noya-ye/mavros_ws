#include "offboard_core_pkg/tasks/corridor_door_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace offboard_core_pkg {

CorridorDoorTask::CorridorDoorTask(
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
    const Config &cfg)
    : logger_(logger), clock_(std::move(clock)), cfg_(cfg),
      ego_(logger, clock_, std::move(goal_pub), cfg.ego) {}

void CorridorDoorTask::onEnter(Context &ctx, MavrosIface &iface) {
  phase_ = Phase::FINDING;
  doors_crossed_ = 0;
  phase_elapsed_s_ = 0.0;
  ctx.fault.clear();
  hold(ctx);
  (void)iface;
}

ITask::Status CorridorDoorTask::tick(Context &ctx, MavrosIface &iface, double dt_s) {
  if (phase_ == Phase::FAILED) return Status::FAILURE;
  const double dt = std::clamp(std::isfinite(dt_s) ? dt_s : 0.0, 0.0, 0.2);
  phase_elapsed_s_ += dt;
  if (phase_elapsed_s_ > cfg_.stage_timeout_s) {
    fail(ctx, "corridor door stage timed out");
    return Status::FAILURE;
  }
  if (!ctx.connected || !ctx.position_valid || !ctx.finitePosition()) {
    hold(ctx);
    return Status::RUNNING;
  }

  if (phase_ == Phase::FINDING) {
    double opening_x = 0.0;
    double opening_y = 0.0;
    if (!mapFresh(ctx) || !findOpening(ctx, opening_x, opening_y)) {
      hold(ctx);
      return Status::RUNNING;
    }
    target_x_ = opening_x;
    target_y_ = opening_y;
    target_z_ = ctx.position_enu.z;
    ego_.setTargetEnu(target_x_, target_y_, target_z_);
    ego_.onEnter(ctx, iface);
    phase_ = Phase::APPROACHING;
    phase_elapsed_s_ = 0.0;
  }

  if (phase_ == Phase::APPROACHING) {
    const auto status = ego_.tick(ctx, iface, dt);
    if (status == Status::FAILURE) {
      fail(ctx, "EGO failed to reach corridor opening");
      return Status::FAILURE;
    }
    if (status == Status::SUCCESS) {
      ego_.onExit(ctx, iface);
      goto_ = GotoTask(target_x_ + cfg_.crossing_distance_m, target_y_, target_z_,
                       cfg_.goto_tolerance_m);
      goto_.onEnter(ctx, iface);
      phase_ = Phase::CROSSING;
      phase_elapsed_s_ = 0.0;
    }
    return Status::RUNNING;
  }

  if (phase_ == Phase::CROSSING) {
    const auto status = goto_.tick(ctx, iface, dt);
    if (status == Status::FAILURE) {
      fail(ctx, "goto failed while crossing corridor door");
      return Status::FAILURE;
    }
    if (status == Status::SUCCESS) {
      ++doors_crossed_;
      RCLCPP_INFO(logger_, "Crossed door %d/%d at x=%.2f y=%.2f",
                  doors_crossed_, cfg_.door_count, target_x_, target_y_);
      if (doors_crossed_ >= cfg_.door_count) {
        hold(ctx);
        return Status::SUCCESS;
      }
      phase_ = Phase::FINDING;
      phase_elapsed_s_ = 0.0;
    }
  }
  return Status::RUNNING;
}

void CorridorDoorTask::onExit(Context &ctx, MavrosIface &iface) {
  if (phase_ == Phase::APPROACHING) ego_.onExit(ctx, iface);
  hold(ctx);
}

bool CorridorDoorTask::mapFresh(const Context &ctx) const {
  if (!ctx.occupancy_grid_valid || !std::isfinite(ctx.occupancy_grid_resolution) ||
      ctx.occupancy_grid_resolution <= 0.0 || ctx.occupancy_grid_width == 0 ||
      ctx.occupancy_grid_height == 0 || ctx.occupancy_grid_stamp_us == 0) {
    return false;
  }
  const auto now_us = static_cast<std::uint64_t>(clock_->now().nanoseconds() / 1000ULL);
  if (now_us < ctx.occupancy_grid_stamp_us) return false;
  return now_us - ctx.occupancy_grid_stamp_us <=
      static_cast<std::uint64_t>(cfg_.occupancy_timeout_s * 1e6);
}

bool CorridorDoorTask::findOpening(const Context &ctx, double &x, double &y) const {
  const double resolution = ctx.occupancy_grid_resolution;
  const int first_x = std::max(0, static_cast<int>(std::floor(
      (ctx.position_enu.x + cfg_.min_lookahead_m - ctx.occupancy_grid_origin_x) /
      resolution)));
  const int last_x = std::min(static_cast<int>(ctx.occupancy_grid_width) - 1,
      static_cast<int>(std::floor((ctx.position_enu.x + cfg_.max_lookahead_m -
          ctx.occupancy_grid_origin_x) / resolution)));
  const int min_cells = std::max(1, static_cast<int>(std::ceil(
      cfg_.min_opening_width_m / resolution)));
  const int max_cells = static_cast<int>(std::floor(
      cfg_.max_opening_width_m / resolution + 1e-9));
  const auto occupied = [&](int ix, int iy) {
    const auto index = static_cast<std::size_t>(iy) * ctx.occupancy_grid_width +
                       static_cast<std::size_t>(ix);
    return index < ctx.occupancy_grid_data.size() &&
           ctx.occupancy_grid_data[index] >= cfg_.occupied_threshold;
  };
  const auto free = [&](int ix, int iy) {
    const auto index = static_cast<std::size_t>(iy) * ctx.occupancy_grid_width +
                       static_cast<std::size_t>(ix);
    return index < ctx.occupancy_grid_data.size() &&
           ctx.occupancy_grid_data[index] >= 0 &&
           ctx.occupancy_grid_data[index] < cfg_.occupied_threshold;
  };

  for (int ix = first_x; ix <= last_x; ++ix) {
    int best_start = -1;
    int best_end = -1;
    double best_distance = INFINITY;
    for (int iy = 0; iy < static_cast<int>(ctx.occupancy_grid_height);) {
      if (!free(ix, iy)) {
        ++iy;
        continue;
      }
      const int start = iy;
      while (iy < static_cast<int>(ctx.occupancy_grid_height) && free(ix, iy)) ++iy;
      const int end = iy - 1;
      const int opening_cells = end - start + 1;
      if (opening_cells < min_cells || opening_cells > max_cells || start == 0 ||
          end + 1 >= static_cast<int>(ctx.occupancy_grid_height) ||
          !occupied(ix, start - 1) || !occupied(ix, end + 1)) {
        continue;
      }
      const double center_y = ctx.occupancy_grid_origin_y +
          (static_cast<double>(start + end + 1) * 0.5) * resolution;
      const double distance = std::abs(center_y - ctx.position_enu.y);
      if (distance < best_distance) {
        best_start = start;
        best_end = end;
        best_distance = distance;
      }
    }
    if (best_start >= 0) {
      x = ctx.occupancy_grid_origin_x + (static_cast<double>(ix) + 0.5) * resolution;
      y = ctx.occupancy_grid_origin_y +
          (static_cast<double>(best_start + best_end + 1) * 0.5) * resolution;
      return true;
    }
  }
  return false;
}

void CorridorDoorTask::hold(Context &ctx) const {
  ctx.position_setpoint_enu = ctx.position_enu;
  ctx.velocity_setpoint_enu = {0.0, 0.0, 0.0};
  ctx.acceleration_setpoint_enu = {0.0, 0.0, 0.0};
  ctx.yaw_setpoint_enu = ctx.home_yaw_enu;
  ctx.setpoint_mode = SetpointMode::POSITION;
  ctx.use_position_velocity_acceleration = false;
  ctx.publish_position_setpoint = true;
}

void CorridorDoorTask::fail(Context &ctx, const char *reason) {
  phase_ = Phase::FAILED;
  ctx.fault = std::string("corridor_door: ") + reason;
  hold(ctx);
  RCLCPP_ERROR(logger_, "%s", ctx.fault.c_str());
}

}  // namespace offboard_core_pkg
