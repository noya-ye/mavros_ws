#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg {

class EgoVelPlanner {
public:
  struct Config {
    double cmd_timeout_s{0.5};
    double odom_timeout_s{0.5};
    double max_step_m{0.10};
    double kp_xy{1.0};
    double kp_z{1.0};
    double max_cmd_xy_m{0.30};
    double max_cmd_z_m{0.15};
    bool use_velocity_ff{true};
    bool use_acceleration_ff{false};
    double vel_ff_scale{0.5};
    double acc_ff_scale{0.0};
    double max_vel_xy_mps{0.5};
    double max_vel_z_mps{0.3};
    double max_acc_xy_mps2{0.6};
    double max_acc_z_mps2{0.4};
    double err_xy_hold_m{2.0};
    double err_z_hold_m{1.0};
    // The current EGO planner is two-dimensional and emits a nominal Z value.
    // Do not treat that value as a flight-altitude command by default.
    bool follow_ego_z{false};
    double x_sign{1.0};
    double y_sign{1.0};
    bool swap_xy{true};
    double yaw_align_rad{0.0};
    bool use_ego_yaw{false};
  };
  enum class Result { OK, HOLD_NO_POSITION, HOLD_STALE_COMMAND, HOLD_STALE_ODOM, HOLD_ERROR };
  struct Debug { double cmd_age_s{999}, odom_age_s{999}, err_xy{0}, err_z{0}; std::string reason; };
  EgoVelPlanner() = default;//让编译器自动生成 EgoVelPlanner 的默认构造函数
  explicit EgoVelPlanner(const Config &cfg) : cfg_(cfg) {}//根据传入的配置参数 cfg 初始化 EgoVelPlanner 对象
  // 方式1： 
  // EgoVelPlanner planner;
  //         ↓
  // EgoVelPlanner()
  //         ↓
  // cfg_ 使用默认配置


  // 方式2：
  // Config cfg;
  // EgoVelPlanner planner(cfg);
  //         ↓
  // EgoVelPlanner(const Config &cfg)
  //         ↓
  // cfg_ = 你传进来的配置
  const Config &config() const { return cfg_; }
  void reset(Context &) { initialized_ = true; }


  //plan() 函数是 EgoVelPlanner 类的核心功能，用于根据当前的上下文（Context）计算出无人机的速度规划，并将结果存储在 Context 中。它还可以输出调试信息。
  Result plan(Context &ctx, Debug *out = nullptr) {
    Debug dbg; const auto now = nowUs();
    dbg.cmd_age_s = age(now, ctx.ego_cmd_stamp_us); dbg.odom_age_s = age(now, ctx.ego_odom_stamp_us);
    if (!ctx.position_valid || !ctx.finitePosition()) { hold(ctx); 
      dbg.reason="position invalid"; 
      if(out)*out=dbg; return Result::HOLD_NO_POSITION; }//位置无效
    if (!ctx.ego_cmd_valid || dbg.cmd_age_s > cfg_.cmd_timeout_s) {
       hold(ctx); 
       dbg.reason="stale EGO command";
        if(out)*out=dbg; return Result::HOLD_STALE_COMMAND; }//EGO命令无效或过期
    if (!ctx.ego_odom_valid || dbg.odom_age_s > cfg_.odom_timeout_s) {
       hold(ctx);
        dbg.reason="stale EGO odometry";
         if(out)*out=dbg; return Result::HOLD_STALE_ODOM; }//EGO里程计无效或过期
    const double ex=ctx.ego_cmd_position.x-ctx.ego_odom_position.x,
     ey=ctx.ego_cmd_position.y-ctx.ego_odom_position.y,
      ez=ctx.ego_cmd_position.z-ctx.ego_odom_position.z;
    dbg.err_xy=std::hypot(ex,ey);
    dbg.err_z=std::fabs(ez);
    if(dbg.err_xy>cfg_.err_xy_hold_m ||
       (cfg_.follow_ego_z && dbg.err_z>cfg_.err_z_hold_m)){
      hold(ctx);
      dbg.reason="EGO error too large";
      if(out)*out=dbg;return Result::HOLD_ERROR;}//ego误差过大，保持当前位置
    double mx,my; map(ex,ey,mx,my); 
    limit(mx,my,cfg_.max_cmd_xy_m);
    ctx.position_setpoint_enu.x=ctx.position_enu.x+cfg_.kp_xy*mx;//这里的误差用的是ego命令与lio里程计的误差，而不是当前位置与目标位置的误差
    ctx.position_setpoint_enu.y=ctx.position_enu.y+cfg_.kp_xy*my;
    ctx.position_setpoint_enu.z=cfg_.follow_ego_z
      ? ctx.position_enu.z+cfg_.kp_z*ez
      : ctx.position_enu.z;
    ctx.velocity_setpoint_enu={0,0,0}; ctx.acceleration_setpoint_enu={0,0,0};
    if(cfg_.use_velocity_ff){
      double vx,vy;
      map(ctx.ego_cmd_velocity.x,ctx.ego_cmd_velocity.y,vx,vy);
      ctx.velocity_setpoint_enu={vx*cfg_.vel_ff_scale,vy*cfg_.vel_ff_scale,
        cfg_.follow_ego_z ? ctx.ego_cmd_velocity.z*cfg_.vel_ff_scale : 0.0};
        limit(ctx.velocity_setpoint_enu.x,ctx.velocity_setpoint_enu.y,cfg_.max_vel_xy_mps);
        ctx.velocity_setpoint_enu.z=
        std::clamp(ctx.velocity_setpoint_enu.z,-cfg_.max_vel_z_mps,cfg_.max_vel_z_mps);}
    if(cfg_.use_acceleration_ff){
      double ax,ay;
      map(ctx.ego_cmd_acceleration.x,ctx.ego_cmd_acceleration.y,ax,ay);//map函数将EGO命令的加速度从相机/里程计坐标系转换为ENU坐标系，并应用配置中的缩放因子和限制条件
      ctx.acceleration_setpoint_enu={ax*cfg_.acc_ff_scale,
        ay*cfg_.acc_ff_scale,
        cfg_.follow_ego_z ? ctx.ego_cmd_acceleration.z*cfg_.acc_ff_scale : 0.0};
        limit(ctx.acceleration_setpoint_enu.x,
          ctx.acceleration_setpoint_enu.y,cfg_.max_acc_xy_mps2);
          ctx.acceleration_setpoint_enu.z=
          std::clamp(ctx.acceleration_setpoint_enu.z,-cfg_.max_acc_z_mps2,cfg_.max_acc_z_mps2);}


    ctx.yaw_setpoint_enu=cfg_.use_ego_yaw?ctx.ego_cmd_yaw:ctx.yaw_enu;

    ctx.setpoint_mode = cfg_.use_acceleration_ff ? SetpointMode::POSITION_VELOCITY_ACCELERATION 
    : (cfg_.use_velocity_ff ? SetpointMode::POSITION_VELOCITY : SetpointMode::POSITION);
    //设置控制方式
    ctx.use_position_velocity_acceleration=false;
     ctx.publish_position_setpoint=true; 
     dbg.reason="OK";
      if(out)*out=dbg;
       return Result::OK;
  }
  static const char *resultName(Result r){switch(r){case Result::OK:return "OK";case Result::HOLD_NO_POSITION:return "HOLD_NO_POSITION";case Result::HOLD_STALE_COMMAND:return "HOLD_STALE_COMMAND";case Result::HOLD_STALE_ODOM:return "HOLD_STALE_ODOM";default:return "HOLD_ERROR";}}
private:
  static std::uint64_t nowUs(){
    return static_cast<std::uint64_t>(rclcpp::Clock().now().nanoseconds()/1000ULL);} static double age(std::uint64_t n,std::uint64_t s){return s==0||n<s?999.0:static_cast<double>(n-s)/1e6;}
  
    void hold(Context &c){
    c.position_setpoint_enu=c.position_enu;
    c.velocity_setpoint_enu={0,0,0};
    c.acceleration_setpoint_enu={0,0,0};
    c.setpoint_mode=SetpointMode::POSITION;
    c.use_position_velocity_acceleration=false;
    c.publish_position_setpoint=true;}//保持当前高度

  void map(double x,double y,double &a,double &b) const {
    if(cfg_.swap_xy)std::swap(x,y);
    const double c=std::cos(cfg_.yaw_align_rad),s=std::sin(cfg_.yaw_align_rad);
    a=cfg_.x_sign*(c*x-s*y);b=cfg_.y_sign*(s*x+c*y);}//坐标转换

  static void limit(double &x,double &y,double m){
    const double n=std::hypot(x,y);
    if(m>0&&n>m){x*=m/n;y*=m/n;}}//限制变化幅度
  
  Config cfg_; 
  bool initialized_{false};
};
}  // namespace offboard_core_pkg
