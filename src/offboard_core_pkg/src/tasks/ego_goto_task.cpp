#include "offboard_core_pkg/tasks/ego_goto_task.hpp"
#include <algorithm>
#include <cmath>

namespace offboard_core_pkg {
void EgoGotoTask::onEnter(Context &ctx, MavrosIface &) {
  started_=false; elapsed_=0; stable_=0; planner_.reset(ctx);
  ctx.ego_cmd_valid=false;
  ctx.setpoint_mode=SetpointMode::POSITION;
  if (ctx.home_initialized) { target_x_=ctx.home_enu.x+cfg_.x_rel; target_y_=ctx.home_enu.y+cfg_.y_rel; target_z_=ctx.home_enu.z+cfg_.height_m; }
  else { target_x_=ctx.position_enu.x+cfg_.x_rel; target_y_=ctx.position_enu.y+cfg_.y_rel; target_z_=ctx.position_enu.z+cfg_.height_m; }
  if (ctx.ego_odom_valid) { publishGoal(ctx); started_=true; }
}

ITask::Status EgoGotoTask::tick(Context &ctx, MavrosIface &, double dt_s) {
  if (!ctx.position_valid || !ctx.finitePosition()) { hold(ctx); return Status::RUNNING; }
  if (!started_) { if (!ctx.ego_odom_valid) { hold(ctx); return Status::RUNNING; } publishGoal(ctx); started_=true; }
  const double dt=std::clamp(std::isfinite(dt_s)?dt_s:0.0,0.0,0.2); elapsed_+=dt;
  if(cfg_.goal_republish_s>0 && elapsed_>=cfg_.goal_republish_s){publishGoal(ctx);elapsed_=0;}
  EgoVelPlanner::Debug dbg; const auto result=planner_.plan(ctx,&dbg);
  if(result!=EgoVelPlanner::Result::OK){stable_=0;return Status::RUNNING;}
  const double ex=ctx.position_enu.x-target_x_, ey=ctx.position_enu.y-target_y_, ez=ctx.position_enu.z-target_z_;
  const double vxy=std::hypot(ctx.velocity_enu.x,ctx.velocity_enu.y);
  if(std::hypot(ex,ey)<=cfg_.arrive_xy_m && std::fabs(ez)<=cfg_.arrive_z_m && vxy<=cfg_.stable_vxy_mps && std::fabs(ctx.velocity_enu.z)<=cfg_.stable_vz_mps) stable_+=dt; else stable_=0;
  if(stable_>=cfg_.stable_required_s){hold(ctx);return Status::SUCCESS;}
  return Status::RUNNING;
}

void EgoGotoTask::onExit(Context &ctx, MavrosIface &) { hold(ctx); }
void EgoGotoTask::publishGoal(const Context &ctx) { 
  if(!goal_pub_ || !ctx.ego_odom_valid) return;
  double ego_dx=0,ego_dy=0; 
  inverseMap(target_x_-ctx.position_enu.x,target_y_-ctx.position_enu.y,ego_dx,ego_dy); 
  target_ego_x_=ctx.ego_odom_position.x+ego_dx; 
  target_ego_y_=ctx.ego_odom_position.y+ego_dy; 
  target_ego_z_=ctx.ego_odom_position.z+(target_z_-ctx.position_enu.z); 
  geometry_msgs::msg::PoseStamped msg; msg.header.stamp=clock_->now(); 
  msg.header.frame_id=cfg_.goal_frame; msg.pose.position.x=target_ego_x_; 
  msg.pose.position.y=target_ego_y_; 
  msg.pose.position.z=target_ego_z_; 
  msg.pose.orientation.w=1.0; 
  goal_pub_->publish(msg); 
}
void EgoGotoTask::inverseMap(double x,double y,double &ox,double &oy) const { 
  const double sx=std::fabs(cfg_.planner.x_sign)>1e-6?cfg_.planner.x_sign:1.0, 
  sy=std::fabs(cfg_.planner.y_sign)>1e-6?cfg_.planner.y_sign:1.0; 
  const double rx=x/sx,
                ry=y/sy,
                c=std::cos(cfg_.planner.yaw_align_rad),
                s=std::sin(cfg_.planner.yaw_align_rad),
                mx=c*rx+s*ry,my=-s*rx+c*ry;
   if(cfg_.planner.swap_xy)
   {ox=my;oy=mx;}
   else{ox=mx;oy=my;} 
  }
void EgoGotoTask::hold(Context &ctx) { 
  ctx.position_setpoint_enu=ctx.position_enu; 
  ctx.velocity_setpoint_enu={0,0,0}; 
  ctx.acceleration_setpoint_enu={0,0,0}; 
  ctx.yaw_setpoint_enu=cfg_.yaw_local; 
  ctx.setpoint_mode=SetpointMode::POSITION; 
  ctx.use_position_velocity_acceleration=false; 
  ctx.publish_position_setpoint=true; }
}  // namespace offboard_core_pkg
