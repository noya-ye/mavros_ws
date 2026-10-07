#include "door_navigation/ros_helpers.hpp"
#include "door_navigation/diagnostic_log.hpp"
#include <door_offboard/msg/planner_enable.hpp>
#include <door_offboard/msg/door_goal.hpp>
#include <door_offboard/msg/goal_reached.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <deque>

namespace door_navigation {
using Goal=door_offboard::msg::DoorGoal;
using Enable=door_offboard::msg::PlannerEnable;
using Reached=door_offboard::msg::GoalReached;
using PoseMsg=geometry_msgs::msg::PoseStamped;
using Steady=std::chrono::steady_clock;
using Json=nlohmann::json;

class FlightNode : public rclcpp::Node {
 public:
  FlightNode() : Node("door_flight") {
    frame_=declare_parameter("local_frame",std::string("map"));
    lio_frame_=declare_parameter("planning_frame",std::string("lidar"));
    child_=declare_parameter("odom_child_frame",std::string("body"));
    const auto odom_topic=declare_parameter("odom_topic",std::string("/fastlio2/lio_odom"));
    height_=number("hover_height_m",0.3); rate_=number("setpoint_rate_hz",20);
    if (rate_<10) throw std::invalid_argument("setpoint_rate_hz must be at least 10");
    prestream_=number("presetpoint_duration_s",2.0); service_timeout_=number("command_timeout_s",10.0);
    retry_=number("command_retry_interval_s",1.0); takeoff_timeout_=number("takeoff_timeout_s",30);
    hover_=number("hover_duration_s",2.0); height_tolerance_=number("height_tolerance_m",0.03);
    input_age_=number("input_timeout_s",0.5); goal_age_=number("goal_timeout_s",0.6);
    sync_=number("alignment_sync_tolerance_s",0.08); align_error_=number("alignment_error_m",0.15);
    arrival_=number("arrival_tolerance_m",0.10); stop_speed_=number("stop_speed_mps",0.05);
    arrive_dwell_=number("arrival_dwell_s",0.3);
    tracking_error_=number("tracking_error_m",0.12);
    cfg_.max_speed=number("max_speed",0.20); cfg_.max_acceleration=number("max_acceleration",0.30);
    const auto offset=declare_parameter("body_offset",std::vector<double>{0,0,0});
    if (offset.size()!=3) throw std::invalid_argument("body_offset");
    body_offset_=Vec3(offset[0],offset[1],offset[2]);
    if (!body_offset_.allFinite()) throw std::invalid_argument("body_offset");
    auto_start_=declare_parameter("auto_start",true);
    log_=std::make_unique<DiagnosticLog>(*this);
    setpoint_pub_=create_publisher<PoseMsg>("/mavros/setpoint_position/local",10);
    target_pub_=create_publisher<PoseMsg>("/door/target_pose",10);
    enable_pub_=create_publisher<Enable>("/door/enable",10);
    reached_pub_=create_publisher<Reached>("/door/reached",10);
    status_pub_=create_publisher<std_msgs::msg::String>("/door/flight_status",10);
    mode_client_=create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_=create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    state_sub_=create_subscription<mavros_msgs::msg::State>("/mavros/state",10,
      [this](mavros_msgs::msg::State::ConstSharedPtr m) { state_=*m; state_received_=Steady::now(); });
    pose_sub_=create_subscription<PoseMsg>("/mavros/local_position/pose",rclcpp::SensorDataQoS(),
      [this](PoseMsg::ConstSharedPtr m) {
        if (m->header.frame_id!=frame_) { sensor_error_="MAVROS local frame mismatch"; return; }
        auto p=read_pose(m->pose,seconds(m->header.stamp),mav_);
        if (!p) return;
        mav_=*p; mav_history_.push_back(*p); if (mav_history_.size()>100) mav_history_.pop_front();
      });
    lio_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic,rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if (m->header.frame_id!=lio_frame_ || m->child_frame_id!=child_) { sensor_error_="LIO frame mismatch"; return; }
        auto msg=m->pose.pose;
        const auto &q=msg.orientation; Eigen::Quaterniond quat(q.w,q.x,q.y,q.z);
        if (!quat.coeffs().allFinite() || quat.norm()<1e-8) { sensor_error_="Invalid LIO orientation"; return; }
        const Vec3 adjusted=vector(msg.position)+quat.normalized()*body_offset_; assign(msg.position,adjusted);
        auto p=read_pose(msg,seconds(m->header.stamp),lio_); if (p) lio_=*p;
      });
    goal_sub_=create_subscription<Goal>("/door/goal",10,[this](Goal::ConstSharedPtr m) {
      if (!activated_ || m->session_id!=session_ || m->header.frame_id!=lio_frame_ ||
          !fresh(seconds(m->header.stamp),goal_age_) || m->goal_id<last_seen_id_) return;
      if (m->valid && (!vector(m->target).allFinite() || m->goal_id==0 ||
          (m->stage!="APPROACH" && m->stage!="CROSS"))) { sensor_error_="Malformed planner goal"; return; }
      if (m->valid && m->goal_id==tracked_id_ && (vector(m->target)-tracked_lio_).norm()>1e-5) {
        sensor_error_="Locked target changed without a new ID"; return;
      }
      last_seen_id_=m->goal_id; goal_=*m; goal_received_=Steady::now();
    });
    phase_started_=Steady::now(); last_tick_=Steady::now();
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1/rate_)),
      [this] { tick(); });
    RCLCPP_INFO(get_logger(),"Direct MAVROS position flight: hover Z %.2f m, %.0f Hz; no landing task",height_,rate_);
  }
 private:
  struct Pose { Vec3 p; double yaw,stamp,speed; Steady::time_point received; };
  Config cfg_;
  std::string frame_,lio_frame_,child_,phase_="WAIT_INPUT",reason_,sensor_error_;
  bool auto_start_,home_=false,takeoff_started_=false,activated_=false,holding_=true,pending_mode_=false,pending_arm_=false;
  double height_,rate_,prestream_,service_timeout_,retry_,takeoff_timeout_,hover_,height_tolerance_;
  double input_age_,goal_age_,sync_,align_error_,arrival_,stop_speed_,arrive_dwell_;
  double tracking_error_,motion_elapsed_=0;
  bool tracking_wait_=false;
  Trajectory samples_;
  Steady::time_point motion_tick_{};
  double fixed_z_=0,yaw_=0,lio_z_=0,alignment_yaw_=0,last_ros_time_=0;
  double last_mode_request_=neg_inf,last_arm_request_=neg_inf;
  Vec3 body_offset_,hold_=Vec3::Zero(),reference_=Vec3::Zero(),tracked_lio_=Vec3::Zero();
  Vec2 translation_=Vec2::Zero(),target_=Vec2::Zero();
  Eigen::Matrix2d rotation_=Eigen::Matrix2d::Identity();
  uint64_t session_=0,tracked_id_=0,reached_id_=0,last_seen_id_=0;
  std::optional<Pose> mav_,lio_;
  std::deque<Pose> mav_history_;
  std::optional<Goal> goal_;
  mavros_msgs::msg::State state_;
  Steady::time_point phase_started_,last_tick_,state_received_{},goal_received_{};
  std::optional<Steady::time_point> stable_since_,arrival_since_;
  std::unique_ptr<DiagnosticLog> log_;
  rclcpp::Publisher<PoseMsg>::SharedPtr setpoint_pub_,target_pub_;
  rclcpp::Publisher<Enable>::SharedPtr enable_pub_;
  rclcpp::Publisher<Reached>::SharedPtr reached_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<Goal>::SharedPtr goal_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  double number(const std::string &name,double value) {
    const double v=declare_parameter(name,value);
    if (!std::isfinite(v) || v<=0) throw std::invalid_argument(name);
    return v;
  }
  double age(Steady::time_point stamp) const { return std::chrono::duration<double>(Steady::now()-stamp).count(); }
  bool fresh(double stamp,double limit) const { const double t=now().seconds(); return t>=stamp && t-stamp<=limit; }
  bool valid_pose(const std::optional<Pose> &p) const {
    return p && fresh(p->stamp,input_age_) && age(p->received)<=input_age_;
  }
  std::optional<Pose> read_pose(const geometry_msgs::msg::Pose &m,double stamp,const std::optional<Pose> &previous) {
    const Vec3 p=vector(m.position); const auto &q=m.orientation; Eigen::Quaterniond quat(q.w,q.x,q.y,q.z);
    if (!p.allFinite() || !quat.coeffs().allFinite() || quat.norm()<1e-8) { sensor_error_="Nonfinite pose"; return {}; }
    if (previous && stamp<=previous->stamp) {
      if (stamp<previous->stamp) sensor_error_="Odometry clock reset";
      return {};
    }
    const auto r=quat.normalized().toRotationMatrix();
    return Pose{p,std::atan2(r(1,0),r(0,0)),stamp,previous?(p-previous->p).norm()/(stamp-previous->stamp):1e6,Steady::now()};
  }
  std::optional<Pose> matched_mav() const {
    if (!valid_pose(lio_) || mav_history_.empty()) return {};
    const auto p=std::min_element(mav_history_.begin(),mav_history_.end(),[this](const Pose &a,const Pose &b) {
      return std::abs(a.stamp-lio_->stamp)<std::abs(b.stamp-lio_->stamp);
    });
    if (std::abs(p->stamp-lio_->stamp)>sync_ || !fresh(p->stamp,input_age_)) return {};
    return *p;
  }
  void phase(const std::string &value,const std::string &reason="") {
    const auto previous=phase_;
    phase_=value; reason_=reason; phase_started_=Steady::now(); stable_since_.reset(); arrival_since_.reset();
    log_->write("phase",{{"previous",previous},{"state",value},{"reason",reason},{"session_id",session_}});
    RCLCPP_INFO(get_logger(),"%s: %s",value.c_str(),reason.c_str());
  }
  void hold(const std::string &reason) {
    const bool changed=!holding_ || reason_!=reason;
    reason_=reason;
    if (!holding_) {
      if (valid_pose(mav_)) hold_.head<2>()=mav_->p.head<2>();
      else hold_.head<2>()=reference_.head<2>();
      hold_.z()=takeoff_started_?fixed_z_:hold_.z();
      holding_=true;
    }
    samples_.clear(); tracking_wait_=false; arrival_since_.reset(); reference_=hold_;
    if (changed) log_->write("hold",{{"reason",reason},{"session_id",session_},{"goal_id",tracked_id_},
      {"position",{hold_.x(),hold_.y(),hold_.z()}}});
  }
  void fault(const std::string &reason) {
    hold(reason); if (phase_!="FAULT_HOLD") phase("FAULT_HOLD",reason);
  }
  void request_mode() {
    const double t=now().seconds();
    if (pending_mode_ || t-last_mode_request_<retry_ || !mode_client_->service_is_ready()) return;
    last_mode_request_=t; pending_mode_=true;
    auto req=std::make_shared<mavros_msgs::srv::SetMode::Request>(); req->custom_mode="OFFBOARD";
    log_->write("offboard_request",{{"mode",req->custom_mode}});
    mode_client_->async_send_request(req,[this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture result) {
      pending_mode_=false; log_->write("offboard_response",{{"accepted",result.get()->mode_sent}});
    });
  }
  void request_arm() {
    const double t=now().seconds();
    if (pending_arm_ || t-last_arm_request_<retry_ || !arm_client_->service_is_ready()) return;
    last_arm_request_=t; pending_arm_=true;
    auto req=std::make_shared<mavros_msgs::srv::CommandBool::Request>(); req->value=true;
    log_->write("arm_request",{{"arm",true}});
    arm_client_->async_send_request(req,[this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture result) {
      pending_arm_=false; log_->write("arm_response",{{"accepted",result.get()->success},{"result",result.get()->result}});
    });
  }
  void enable() {
    if (!session_) return;
    Enable m; m.header=header(*this,lio_frame_); m.session_id=session_; m.enabled=phase_=="NAVIGATE";
    m.flight_z_lio=lio_z_; enable_pub_->publish(m);
  }
  void reached() {
    Reached msg; msg.header=header(*this,lio_frame_); msg.session_id=session_; msg.goal_id=reached_id_;
    reached_pub_->publish(msg);
  }
  void navigate() {
    const auto match=matched_mav();
    if (!match) { hold("Waiting for paired LIO/MAVROS poses"); return; }
    const double yaw_error=std::remainder(match->yaw-lio_->yaw-alignment_yaw_,2*pi);
    if ((rotation_*lio_->p.head<2>()+translation_-match->p.head<2>()).norm()>align_error_ || std::abs(yaw_error)>0.2) {
      fault("LIO/MAVROS alignment changed; no automatic recalibration in flight"); return;
    }
    if (!goal_ || age(goal_received_)>goal_age_ || !fresh(seconds(goal_->header.stamp),goal_age_)) {
      hold("No fresh planner goal"); return;
    }
    if (goal_->complete) { hold("All configured doors completed"); phase("COMPLETE_HOLD",reason_); return; }
    if (!goal_->valid) { hold(goal_->reason.empty()?"Waiting for a clear path":goal_->reason); return; }
    if (goal_->goal_id==reached_id_) { hold("Waiting for next goal"); reached(); return; }
    if (goal_->goal_id!=reached_id_+1) { fault("Unexpected goal sequence"); return; }
    if (std::abs(goal_->target.z-lio_z_)>1e-5) { fault("Planner goal height mismatch"); return; }
    const Vec2 mapped=rotation_*vector(goal_->target).head<2>()+translation_;
    PoseMsg target; target.header=header(*this,frame_); target.pose.position=point(mapped,fixed_z_);
    target.pose.orientation=quaternion(yaw_); target_pub_->publish(target);
    if (holding_ || tracked_id_!=goal_->goal_id) {
      if (mav_->speed>stop_speed_ || std::abs(mav_->p.z()-fixed_z_)>height_tolerance_) { hold("Waiting to stop at fixed height"); return; }
      tracked_id_=goal_->goal_id; tracked_lio_=vector(goal_->target); target_=mapped;
      samples_=trajectory({mav_->p.head<2>(),target_},cfg_);
      motion_elapsed_=0; motion_tick_=Steady::now(); tracking_wait_=false;
      reference_=Vec3(mav_->p.x(),mav_->p.y(),fixed_z_);
      holding_=false; arrival_since_.reset(); reason_.clear();
      log_->write("target",{{"session_id",session_},{"goal_id",tracked_id_},{"stage",goal_->stage},
        {"lio_target",{tracked_lio_.x(),tracked_lio_.y(),tracked_lio_.z()}},
        {"start",{mav_->p.x(),mav_->p.y(),mav_->p.z()}},
        {"execution","paced_position"},{"duration_s",samples_.empty()?0:samples_.back().t},
        {"position",{target_.x(),target_.y(),fixed_z_}},{"frame_id",frame_}});
    }
    const bool arrived=(mav_->p.head<2>()-target_).norm()<=arrival_ &&
      std::abs(mav_->p.z()-fixed_z_)<=height_tolerance_ && mav_->speed<=stop_speed_;
    if (arrived) {
      if (!arrival_since_) arrival_since_=Steady::now();
      if (age(*arrival_since_)>=arrive_dwell_) {
        log_->write("reached",{{"session_id",session_},{"goal_id",tracked_id_},
          {"xy_error_m",(mav_->p.head<2>()-target_).norm()},
          {"z_error_m",mav_->p.z()-fixed_z_},{"speed_mps",mav_->speed}});
        reached_id_=tracked_id_; hold("Target reached"); reached(); return;
      }
    } else arrival_since_.reset();
    if (std::abs(mav_->p.z()-fixed_z_)>height_tolerance_*2) { hold("Fixed height error exceeded"); return; }
    // Advance trajectory time only when the aircraft can follow the next point.
    // Waiting keeps the last reference and the original trajectory intact.
    const double dt=std::min(age(motion_tick_),0.1); motion_tick_=Steady::now();
    const double next_time=motion_elapsed_+dt;
    Vec2 wanted=target_;
    if (!samples_.empty()) {
      const auto next=std::upper_bound(samples_.begin(),samples_.end(),next_time,
        [](double t,const Sample &s) { return t<s.t; });
      wanted=next==samples_.end()?target_:(next==samples_.begin()?next->p.head<2>().eval():
        interpolate(*(next-1),*next,next_time).p.head<2>().eval());
    }
    const double error=(mav_->p.head<2>()-wanted).norm();
    if (error>tracking_error_) {
      if (!tracking_wait_) log_->write("tracking_wait",{{"session_id",session_},{"goal_id",tracked_id_},
        {"xy_error_m",error},{"trajectory_time_s",motion_elapsed_},
        {"reference",{reference_.x(),reference_.y(),fixed_z_}}});
      tracking_wait_=true; reason_="Waiting for position tracking"; return;
    }
    if (tracking_wait_) log_->write("tracking_resumed",{{"session_id",session_},{"goal_id",tracked_id_},
      {"trajectory_time_s",motion_elapsed_},{"xy_error_m",error}});
    tracking_wait_=false; reason_.clear(); motion_elapsed_=next_time;
    reference_=Vec3(wanted.x(),wanted.y(),fixed_z_);
  }
  void tick() {
    const auto tick_time=Steady::now(); const double t=now().seconds();
    if (home_ && (t<last_ros_time_ || age(last_tick_)>input_age_)) fault("Control clock/timer discontinuity");
    last_ros_time_=t; last_tick_=tick_time;
    if (phase_=="WAIT_INPUT") {
      if (auto_start_ && state_.connected && age(state_received_)<=input_age_ && valid_pose(mav_) && valid_pose(lio_)) {
        if (!sensor_error_.empty()) { reason_=sensor_error_; }
        else if (state_.armed) { phase("MANUAL","Already armed at startup; restart only before takeoff"); }
        else {
          hold_=mav_->p; reference_=hold_; yaw_=mav_->yaw; home_=true; fixed_z_=height_;
          phase("PRESTREAM");
        }
      }
    } else if (phase_!="MANUAL" && home_) {
      if (!sensor_error_.empty()) fault(sensor_error_);
      if (!valid_pose(mav_) || !valid_pose(lio_) || age(state_received_)>input_age_ || !state_.connected)
        fault("Stale pose/state or disconnected MAVROS");
      // After OFFBOARD acquisition, a pilot mode change is terminal for this run.
      if (state_.connected && age(state_received_)<=input_age_ &&
          ((phase_=="ARM" && state_.mode!="OFFBOARD") ||
           (takeoff_started_ && (state_.mode!="OFFBOARD" || !state_.armed)))) {
        phase("MANUAL","Pilot/autopilot left OFFBOARD or disarmed; no automatic re-entry");
      }
      if (phase_=="PRESTREAM" && age(phase_started_)>=prestream_) phase("OFFBOARD");
      if (phase_=="OFFBOARD") {
        if (state_.mode=="OFFBOARD") phase("ARM");
        else if (age(phase_started_)>service_timeout_) fault("OFFBOARD request timed out");
        else request_mode();
      }
      if (phase_=="ARM") {
        if (state_.armed) {
          takeoff_started_=true; hold_.z()=fixed_z_; reference_=hold_; phase("TAKEOFF");
        } else if (age(phase_started_)>service_timeout_) fault("Arming timed out");
        else request_arm();
      }
      if (phase_=="TAKEOFF" || phase_=="STABILIZE") {
        reference_=hold_;
        if (age(phase_started_)>takeoff_timeout_) fault("Takeoff/stabilization timed out");
        else if (std::abs(mav_->p.z()-fixed_z_)<=height_tolerance_ &&
                 (mav_->p.head<2>()-hold_.head<2>()).norm()<=arrival_ && mav_->speed<=stop_speed_ && lio_->speed<=stop_speed_) {
          if (phase_=="TAKEOFF") phase("STABILIZE");
          if (!stable_since_) stable_since_=Steady::now();
          if (age(*stable_since_)>=hover_) {
            const auto match=matched_mav();
            if (match) {
              alignment_yaw_=match->yaw-lio_->yaw; rotation_=basis(alignment_yaw_);
              translation_=match->p.head<2>()-rotation_*lio_->p.head<2>();
              lio_z_=lio_->p.z()+fixed_z_-match->p.z();
              session_=static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
              activated_=true; phase("NAVIGATE","Stable hover; enabling perception");
              log_->write("alignment",{{"yaw",alignment_yaw_},{"translation",{translation_.x(),translation_.y()}},
                {"fixed_z",fixed_z_},{"flight_z_lio",lio_z_},{"session_id",session_}});
            }
          }
        } else stable_since_.reset();
      }
      if (phase_=="NAVIGATE") navigate();
    }
    enable();
    if (home_ && phase_!="MANUAL") {
      PoseMsg msg; msg.header=header(*this,frame_); assign(msg.pose.position,reference_); msg.pose.orientation=quaternion(yaw_);
      setpoint_pub_->publish(msg);
    }
    Json data={{"state",phase_},{"reason",reason_},{"fixed_z",fixed_z_},{"hover_height_m",height_},
      {"session_id",session_},{"goal_id",tracked_id_},{"reached_id",reached_id_},{"holding",holding_},
      {"setpoint",{reference_.x(),reference_.y(),reference_.z()}},{"stamp",t}};
    data["mavros"]={{"connected",state_.connected},{"armed",state_.armed},{"mode",state_.mode},
      {"state_receive_age_s",state_received_==Steady::time_point{}?Json(nullptr):Json(age(state_received_))},
      {"setpoint_published",home_ && phase_!="MANUAL"},{"frame_id",frame_}};
    if (mav_) data["mavros_pose"]={{"position",{mav_->p.x(),mav_->p.y(),mav_->p.z()}},
      {"yaw",mav_->yaw},{"speed_mps",mav_->speed},{"stamp_age_s",t-mav_->stamp},
      {"receive_age_s",age(mav_->received)},{"reference_error_m",(mav_->p-reference_).norm()},
      {"z_error_m",mav_->p.z()-fixed_z_}};
    if (lio_) data["lio_pose"]={{"position",{lio_->p.x(),lio_->p.y(),lio_->p.z()}},
      {"yaw",lio_->yaw},{"speed_mps",lio_->speed},{"stamp_age_s",t-lio_->stamp},
      {"receive_age_s",age(lio_->received)},{"frame_id",lio_frame_}};
    if (goal_) data["planner"]={{"goal_id",goal_->goal_id},{"valid",goal_->valid},
      {"complete",goal_->complete},{"stage",goal_->stage},{"reason",goal_->reason},
      {"target",{goal_->target.x,goal_->target.y,goal_->target.z}},
      {"stamp_age_s",t-seconds(goal_->header.stamp)},{"receive_age_s",age(goal_received_)}};
    if (activated_ && mav_) data["target_xy_error_m"]=(mav_->p.head<2>()-target_).norm();
    std_msgs::msg::String status; status.data=data.dump(); status_pub_->publish(status); log_->write("status",data);
  }
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<door_navigation::FlightNode>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("door_flight"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
