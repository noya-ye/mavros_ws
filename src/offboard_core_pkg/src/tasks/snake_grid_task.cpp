#include "offboard_core_pkg/tasks/snake_grid_task.hpp"
#include "offboard_core_pkg/mavros_iface.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <queue>

namespace offboard_core_pkg {
namespace { std::atomic<std::uint32_t> next_plan_id{1}; }

SnakeGridTask::SnakeGridTask(rclcpp::Logger logger, const Config &cfg) : logger_(logger), cfg_(cfg) {}
std::string SnakeGridTask::name() const { return "SNAKE_GRID"; }

void SnakeGridTask::onEnter(Context &ctx, MavrosIface &) {
  waypoints_.clear(); route_cells_.clear(); index_ = 0; hover_elapsed_s_ = 0;
  phase_ = Phase::MOVING; plan_ready_ = false; plan_id_ = 0;
  if (!ctx.position_valid || !ctx.finitePosition()) { phase_ = Phase::FAILED; RCLCPP_ERROR(logger_, "[SNAKE] local position invalid"); return; }
  if (cfg_.x_cells <= 0 || cfg_.y_cells <= 0 || !std::isfinite(cfg_.cell_size) || cfg_.cell_size <= 0 ||
      (cfg_.x_sign != 1 && cfg_.x_sign != -1) || (cfg_.y_sign != 1 && cfg_.y_sign != -1) ||
      cfg_.hover_s < 0 || cfg_.max_step_m < 0 || cfg_.arrive_xy_m <= 0 || cfg_.arrive_z_m <= 0) {
    phase_ = Phase::FAILED; RCLCPP_ERROR(logger_, "[SNAKE] invalid configuration"); return;
  }
  origin_x_ = ctx.position_enu.x; origin_y_ = ctx.position_enu.y; target_z_ = ctx.position_enu.z;
  cmd_x_ = origin_x_; cmd_y_ = origin_y_; cmd_z_ = target_z_; yaw_hold_ = ctx.yaw_enu;
  buildWaypoints();
  if (phase_ == Phase::FAILED) return;
  rebuildRouteCells(); plan_id_ = next_plan_id.fetch_add(1); plan_ready_ = true;
  if (waypoints_.empty()) phase_ = Phase::FINISHED;
}

ITask::Status SnakeGridTask::tick(Context &ctx, MavrosIface &, double dt_s) {
  if (phase_ == Phase::FAILED) return Status::FAILURE;
  if (phase_ == Phase::FINISHED) return Status::SUCCESS;
  if (index_ >= waypoints_.size()) { phase_ = Phase::FINISHED; return Status::SUCCESS; }
  const auto &wp = waypoints_[index_];
  if (!ctx.position_valid || !ctx.finitePosition()) { publishSetpoint(ctx); return Status::RUNNING; }
  if (phase_ == Phase::MOVING) {
    moveCommandToward(wp, dt_s); publishSetpoint(ctx);
    if (!wp.hover_after) {
      if (std::hypot(wp.x - cmd_x_, wp.y - cmd_y_) < 1e-4 && std::fabs(wp.z - cmd_z_) < 1e-4) nextWaypoint();
    } else if (arrived(ctx, wp)) {
      cmd_x_ = wp.x; cmd_y_ = wp.y; cmd_z_ = wp.z;
      if (cfg_.hover_s > 1e-3) { hover_elapsed_s_ = 0; phase_ = Phase::HOVERING; } else nextWaypoint();
    }
  } else if (phase_ == Phase::HOVERING) {
    cmd_x_ = wp.x; cmd_y_ = wp.y; cmd_z_ = wp.z; publishSetpoint(ctx);
    if (std::isfinite(dt_s) && dt_s > 0) hover_elapsed_s_ += std::min(dt_s, 0.2);
    if (hover_elapsed_s_ >= cfg_.hover_s) nextWaypoint();
  }
  return Status::RUNNING;
}

void SnakeGridTask::onExit(Context &, MavrosIface &) {}
void SnakeGridTask::onPause(Context &ctx, MavrosIface &) { if (ctx.position_valid) { cmd_x_=ctx.position_enu.x; cmd_y_=ctx.position_enu.y; cmd_z_=ctx.position_enu.z; publishSetpoint(ctx); } hover_elapsed_s_=0; }
void SnakeGridTask::onResume(Context &ctx, MavrosIface &) {
  if (phase_ == Phase::FAILED || phase_ == Phase::FINISHED || waypoints_.empty() || !ctx.position_valid) return;
  const std::size_t end = std::min(index_, waypoints_.size()-1); const std::size_t begin = end > 12 ? end-12 : 0;
  std::size_t nearest=begin; double distance=std::numeric_limits<double>::max();
  for (std::size_t i=begin; i<=end; ++i) { const auto d=std::hypot(waypoints_[i].x-ctx.position_enu.x, waypoints_[i].y-ctx.position_enu.y); if (d<distance) { distance=d; nearest=i; } }
  index_=nearest+1; cmd_x_=ctx.position_enu.x; cmd_y_=ctx.position_enu.y; cmd_z_=ctx.position_enu.z; hover_elapsed_s_=0; phase_=index_>=waypoints_.size()?Phase::FINISHED:Phase::MOVING;
}

void SnakeGridTask::buildWaypoints() {
  std::vector<Waypoint> original; waypoints_.clear();
  if (cfg_.first_axis == FirstAxis::X_FIRST) buildXFirstWaypoints(); else buildYFirstWaypoints();
  original = std::move(waypoints_); waypoints_.clear();
  const int total=cfg_.x_cells*cfg_.y_cells; std::vector<unsigned char> blocked(static_cast<std::size_t>(total));
  auto idx=[this](int x,int y){return y*cfg_.x_cells+x;};
  for (const auto &o: cfg_.obstacle_cells) { if(o.ix<0||o.ix>=cfg_.x_cells||o.iy<0||o.iy>=cfg_.y_cells){phase_=Phase::FAILED;return;} blocked[idx(o.ix,o.iy)]=1; }
  if (blocked[0]) { phase_=Phase::FAILED; return; }
  auto append=[this](const Waypoint &wp){ if(!waypoints_.empty()&&waypoints_.back().ix==wp.ix&&waypoints_.back().iy==wp.iy) waypoints_.back().hover_after|=wp.hover_after; else waypoints_.push_back(wp); };
  if(cfg_.include_start_cell) append(makeWaypoint(0,0,cfg_.stop_mode==StopMode::EVERY_CELL));
  int cx=0,cy=0;
  std::size_t route_index = 0;
  while (route_index < original.size()) {
    const auto &target = original[route_index];
    if (target.ix == 0 && target.iy == 0) { ++route_index; continue; }
    if (!blocked[idx(target.ix,target.iy)]) { append(target); cx=target.ix; cy=target.iy; ++route_index; continue; }
    append(makeWaypoint(cx,cy,true));
    std::size_t n = route_index + 1;
    while(n < original.size() && blocked[idx(original[n].ix,original[n].iy)]) ++n;
    if(n >= original.size()) break;
    const int goal_x=original[n].ix, goal_y=original[n].iy;
    std::vector<int> parent(static_cast<std::size_t>(total), -1); std::queue<std::pair<int,int>> open;
    parent[idx(cx,cy)] = idx(cx,cy); open.push({cx,cy});
    const int dx4[4]={1,-1,0,0}, dy4[4]={0,0,1,-1};
    while(!open.empty()) { const auto [x,y]=open.front(); open.pop(); if(x==goal_x&&y==goal_y) break; for(int k=0;k<4;++k){int nx=x+dx4[k],ny=y+dy4[k]; if(nx<0||nx>=cfg_.x_cells||ny<0||ny>=cfg_.y_cells||blocked[idx(nx,ny)]||parent[idx(nx,ny)]!=-1) continue; parent[idx(nx,ny)]=idx(x,y); open.push({nx,ny});} }
    if(parent[idx(goal_x,goal_y)]==-1){phase_=Phase::FAILED;waypoints_.clear();return;}
    std::vector<std::pair<int,int>> path; for(int x=goal_x,y=goal_y;;){path.push_back({x,y}); if(x==cx&&y==cy) break; const int p=parent[idx(x,y)]; x=p%cfg_.x_cells; y=p/cfg_.x_cells;} std::reverse(path.begin(),path.end());
    for(std::size_t i=1;i<path.size();++i){bool turn=i+1<path.size()&&((path[i].first-path[i-1].first)!=(path[i+1].first-path[i].first)||(path[i].second-path[i-1].second)!=(path[i+1].second-path[i].second)); append(makeWaypoint(path[i].first,path[i].second,turn||i+1==path.size()));} cx=goal_x; cy=goal_y; route_index=n+1;
  }
}
void SnakeGridTask::buildXFirstWaypoints(){for(int y=0;y<cfg_.y_cells;++y)for(int k=0;k<cfg_.x_cells;++k){int x=y%2?cfg_.x_cells-1-k:k;pushWaypoint(x,y,k==cfg_.x_cells-1);}}
void SnakeGridTask::buildYFirstWaypoints(){for(int x=0;x<cfg_.x_cells;++x)for(int k=0;k<cfg_.y_cells;++k){int y=x%2?cfg_.y_cells-1-k:k;pushWaypoint(x,y,k==cfg_.y_cells-1);}}
void SnakeGridTask::pushWaypoint(int x,int y,bool end){if(!cfg_.include_start_cell&&x==0&&y==0)return;waypoints_.push_back(makeWaypoint(x,y,cfg_.stop_mode==StopMode::EVERY_CELL||end));}
SnakeGridTask::Waypoint SnakeGridTask::makeWaypoint(int x,int y,bool hover) const {return {x,y,origin_x_+cfg_.x_sign*x*cfg_.cell_size,origin_y_+cfg_.y_sign*y*cfg_.cell_size,target_z_,hover};}
void SnakeGridTask::rebuildRouteCells(){for(const auto &w:waypoints_)route_cells_.push_back(cellToName(w.ix,w.iy));}
std::string SnakeGridTask::cellToName(int x,int y) const{return "A"+std::to_string(cfg_.x_cells-x)+"B"+std::to_string(y+1);}
void SnakeGridTask::moveCommandToward(const Waypoint &w,double dt){double dx=w.x-cmd_x_,dy=w.y-cmd_y_,dz=w.z-cmd_z_,d=std::sqrt(dx*dx+dy*dy+dz*dz);if(d<1e-6){cmd_x_=w.x;cmd_y_=w.y;cmd_z_=w.z;return;}double t=std::isfinite(dt)&&dt>0?std::min(dt,0.1):0.05;double step=cfg_.max_step_m/0.05*t;if(step>=d||step<1e-6){cmd_x_=w.x;cmd_y_=w.y;cmd_z_=w.z;}else{cmd_x_+=dx*step/d;cmd_y_+=dy*step/d;cmd_z_+=dz*step/d;}}
bool SnakeGridTask::arrived(const Context &c,const Waypoint &w) const{return std::hypot(c.position_enu.x-w.x,c.position_enu.y-w.y)<=cfg_.arrive_xy_m&&std::fabs(c.position_enu.z-w.z)<=cfg_.arrive_z_m;}
void SnakeGridTask::publishSetpoint(Context &c){c.position_setpoint_enu={cmd_x_,cmd_y_,cmd_z_};c.yaw_setpoint_enu=yaw_hold_;c.publish_position_setpoint=true;}
void SnakeGridTask::nextWaypoint(){if(++index_>=waypoints_.size())phase_=Phase::FINISHED;else phase_=Phase::MOVING;hover_elapsed_s_=0;}
std::size_t SnakeGridTask::displayWaypointIndex() const{return waypoints_.empty()?0:std::min(index_+1,waypoints_.size());}
std::uint32_t SnakeGridTask::planId() const{return plan_id_;} bool SnakeGridTask::planReady() const{return plan_ready_;}
const std::vector<std::string>& SnakeGridTask::routeCells() const{return route_cells_;} std::size_t SnakeGridTask::currentIndex() const{return std::min(index_,route_cells_.size());}
std::size_t SnakeGridTask::totalWaypoints() const{return route_cells_.size();} std::string SnakeGridTask::currentCell() const{return route_cells_.empty()?std::string{}:route_cells_[std::min(index_,route_cells_.size()-1)];}
bool SnakeGridTask::finished() const{return phase_==Phase::FINISHED;} bool SnakeGridTask::failed() const{return phase_==Phase::FAILED;}

bool SnakeGridTask::waypointAt(std::size_t index, WaypointInfo &out) const {
  if (index >= waypoints_.size()) return false;
  const auto &waypoint = waypoints_[index];
  out.ix = waypoint.ix;
  out.iy = waypoint.iy;
  out.x = waypoint.x;
  out.y = waypoint.y;
  out.z = waypoint.z;
  return true;
}

void SnakeGridTask::skipCurrentWaypoint() {
  if (index_ >= waypoints_.size()) {
    phase_ = Phase::FINISHED;
    return;
  }
  ++index_;
  hover_elapsed_s_ = 0.0;
  phase_ = index_ >= waypoints_.size() ? Phase::FINISHED : Phase::MOVING;
}

void SnakeGridTask::resumeAfterWaypoint(
  std::size_t completed_index, Context &ctx) {
  if (waypoints_.empty()) {
    phase_ = Phase::FINISHED;
    return;
  }
  index_ = std::min(completed_index + 1, waypoints_.size());
  cmd_x_ = ctx.position_enu.x;
  cmd_y_ = ctx.position_enu.y;
  cmd_z_ = ctx.position_enu.z;
  hover_elapsed_s_ = 0.0;
  phase_ = index_ >= waypoints_.size() ? Phase::FINISHED : Phase::MOVING;
}
}  // namespace offboard_core_pkg
