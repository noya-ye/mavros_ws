#include "door_navigation/cloud.hpp"
#include "door_navigation/ros_helpers.hpp"
#include "door_navigation/diagnostic_log.hpp"
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <rclcpp/create_timer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>

namespace door_navigation {
using Json=nlohmann::json;
using Odom=nav_msgs::msg::Odometry;
using Cloud=sensor_msgs::msg::PointCloud2;
using Motion=trajectory_msgs::msg::MultiDOFJointTrajectory;

class DoorNode : public rclcpp::Node {
 public:
  explicit DoorNode(const rclcpp::NodeOptions &options=rclcpp::NodeOptions()) : Node("door_navigation",options) {
    cloud_topic_=declare_parameter("cloud_topic",std::string("/cloud_registered"));
    odom_topic_=declare_parameter("odom_topic",std::string("/Odometry"));
    frame_=declare_parameter("planning_frame",std::string("camera_init"));
    child_=declare_parameter("odom_child_frame",std::string("body"));
    preview_=declare_parameter("preview_mode",true);
    follow_height_=declare_parameter("preview_follow_height",true);
    require_ready_=declare_parameter("require_control_ready",false);
    auto_z_=declare_parameter("auto_flight_z",false);
    flight_z_=declare_parameter("flight_z",-999.0);
    if (!preview_ && auto_z_ && !require_ready_)
      throw std::invalid_argument("auto_flight_z requires require_control_ready to avoid capturing takeoff height");
    if (!(preview_ && follow_height_) && !(!preview_ && auto_z_) &&
        (!std::isfinite(flight_z_) || flight_z_==-999.0))
      throw std::invalid_argument("Set flight_z explicitly in the gravity-aligned planning frame");
    below_=number("body_below",0.15,true); above_=number("body_above",0.15,true);
    vertical_margin_=number("vertical_margin",0.08,true);
    slice_half_=number("slice_half_thickness",0.08); cache_seconds_=number("cache_seconds",0.6);
    max_age_=number("max_age",0.3); sync_tolerance_=number("sync_tolerance",0.03);
    self_radius_=number("self_filter_radius",0.28,true);
    start_speed_=number("start_speed_limit",0.05); height_tolerance_=number("height_tolerance",0.05);
    observation_tolerance_=number("observation_tolerance",0.08);
    ready_timeout_=number("control_ready_timeout",0.3);
    stable_frames_=declare_parameter("stable_frames",3);
    if (stable_frames_<1) throw std::invalid_argument("stable_frames must be positive");
    body_offset_=offset("body_offset"); lidar_offset_=offset("lidar_offset");
    cfg_.radius=number("radius",0.25); cfg_.margin=number("margin",0.0,true);
    // Perception is forward-only. Retain only the footprint-sized rear raster
    // buffer needed to represent a route starting at the vehicle reference.
    cfg_.x_min=-(cfg_.radius+cfg_.margin+cfg_.resolution);
    cfg_.corridor_width=number("corridor_width",2.2);
    cfg_.corridor_tolerance=number("corridor_tolerance",0.50);
    cfg_.door_min=number("door_min",0.50); cfg_.door_max=number("door_max",1.20);
    const double lateral_range=number("lateral_range",2.0);
    cfg_.y_min=-lateral_range; cfg_.y_max=lateral_range;
    cfg_.max_speed=number("max_speed",0.30); cfg_.max_acceleration=number("max_acceleration",0.30);
    cfg_.pre_distance=number("pre_distance",0.50); cfg_.post_distance=number("post_distance",0.50);
    cfg_.wall_thickness=number("wall_thickness",0.15,true); cfg_.validate();
    log_=std::make_unique<DiagnosticLog>(*this);
    log_->write("planner_model",{{"revision","straight-two-gates-v1"},{"forward_only",true},
      {"path_kind","geometric_reference"},{"max_passes",2}});
    status_pub_=create_publisher<std_msgs::msg::String>("/door/status",10);
    detection_pub_=create_publisher<std_msgs::msg::String>("/door/detection",10);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/door/path",10);
    motion_pub_=create_publisher<Motion>("/door/trajectory",10);
    image_pub_=create_publisher<sensor_msgs::msg::Image>("/door/debug_image",rclcpp::SensorDataQoS());
    odom_sub_=create_subscription<Odom>(odom_topic_,rclcpp::SensorDataQoS(),[this](Odom::ConstSharedPtr m) { on_odom(*m); });
    cloud_sub_=create_subscription<Cloud>(cloud_topic_,rclcpp::SensorDataQoS(),[this](Cloud::ConstSharedPtr m) {
      if (m->header.frame_id!=frame_) { invalidate("Cloud must be registered in planning_frame"); return; }
      pending_.push_back(m); if (pending_.size()>10) pending_.pop_front();
    });
    ready_sub_=create_subscription<std_msgs::msg::Bool>("/door/control_ready",rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          ready_=m->data; ready_stamp_=now().seconds();
          if (!ready_) { captured_z_.reset(); capture_start_=neg_inf; }
          if (!preview_ && require_ready_ && !ready_ && active_) invalidate("Controller handoff withdrawn");
        });
    timer_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_seconds(0.2),[this]() {
      tick_started_=std::chrono::steady_clock::now(); tick(); tick_started_.reset();
    });
    RCLCPP_INFO(get_logger(),"C++ door planner: preview=%s, inflation=%.2f m",preview_?"true":"false",cfg_.radius+cfg_.margin);
  }
 private:
  struct Pose { double stamp; Vec3 body, origin; Eigen::Matrix3d rotation; double speed; };
  struct Frame { double stamp; std::vector<Vec3> points; Vec3 origin; };
  struct Active { Trajectory samples; double start; };
  struct Target { Gate gate; bool crossing=false; };
  Config cfg_;
  std::string cloud_topic_,odom_topic_,frame_,child_;
  bool preview_,follow_height_,require_ready_,auto_z_,ready_=false;
  std::optional<double> captured_z_;
  double capture_start_=neg_inf,capture_z_=0;
  double flight_z_,below_,above_,vertical_margin_,slice_half_,cache_seconds_,max_age_,sync_tolerance_;
  double self_radius_,start_speed_,height_tolerance_,observation_tolerance_,ready_timeout_;
  double last_cloud_=neg_inf,ready_stamp_=neg_inf;
  int stable_frames_,count_=0;
  Vec3 body_offset_,lidar_offset_;
  std::deque<Pose> odoms_;
  std::deque<Cloud::ConstSharedPtr> pending_;
  std::deque<Frame> frames_;
  std::optional<Pose> latest_;
  std::optional<double> heading_;
  std::optional<Vec2> candidate_;
  std::optional<Active> active_;
  std::optional<Target> target_;
  bool segment_clear_=false;
  std::string plan_stage_;
  Route passed_;
  std::unique_ptr<DiagnosticLog> log_;
  std::optional<std::chrono::steady_clock::time_point> tick_started_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,detection_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<Motion>::SharedPtr motion_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double number(const std::string &name,double value,bool zero=false) {
    const double v=declare_parameter(name,value);
    if (!std::isfinite(v) || (zero?v<0:v<=0)) throw std::invalid_argument("Invalid parameter: "+name);
    return v;
  }
  Vec3 offset(const std::string &name) {
    const auto v=declare_parameter(name,std::vector<double>{0,0,0});
    if (v.size()!=3) throw std::invalid_argument(name+" must contain three values");
    Vec3 out(v[0],v[1],v[2]);
    if (!out.allFinite()) throw std::invalid_argument("Nonfinite "+name);
    return out;
  }
  void status(const std::string &state,const std::string &reason="",bool valid=false) {
    std_msgs::msg::String m;
    m.data=Json{{"state",state},{"reason",reason},{"trajectory_valid",valid && !preview_},
                {"plan_stage",plan_stage_},
                {"path_kind","geometric_reference"},{"segment_clear",segment_clear_},
                {"target_locked",target_.has_value()},{"passed_count",passed_.size()},
                {"preview_mode",preview_},{"stamp",now().seconds()},
                {"locked_flight_z",captured_z_ ? Json(*captured_z_) : Json(nullptr)}}.dump();
    status_pub_->publish(m); 
    if (!log_->enabled()) return;
    Json record=Json::parse(m.data);
    record["odom_stamp"]=latest_ ? Json(latest_->stamp) : Json(nullptr);
    record["cloud_stamp"]=std::isfinite(last_cloud_) ? Json(last_cloud_) : Json(nullptr);
    record["control_ready"]=ready_;
    record["control_stamp"]=std::isfinite(ready_stamp_) ? Json(ready_stamp_) : Json(nullptr);
    record["cached_frames"]=frames_.size();
    record["processing_ms"]=tick_started_ ? Json(std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-*tick_started_).count()) : Json(nullptr);
    if (latest_) {
      record["position"]={latest_->body.x(),latest_->body.y(),latest_->body.z()};
      record["speed"]=latest_->speed;
    }
    log_->write("status",record);
  }
  void clear_paths() {
    path_pub_->publish(make_path({},0,0,header(*this,frame_))); 
  }
  void invalidate(const std::string &reason) {
    plan_stage_.clear();
    segment_clear_=false;
    capture_start_=neg_inf;
    if (active_) { Motion empty; empty.header=header(*this,frame_); motion_pub_->publish(empty); }
    active_.reset(); candidate_.reset(); count_=0; clear_paths();  status("INVALID",reason);
  }
  void on_odom(const Odom &m) {
    if (m.header.frame_id!=frame_ || m.child_frame_id!=child_) {
      target_.reset(); passed_.clear(); heading_.reset();
      latest_.reset(); invalidate("Odometry frame mismatch; configure explicit frame names"); return;
    }
    const auto &q=m.pose.pose.orientation;
    Eigen::Quaterniond quat(q.w,q.x,q.y,q.z);
    if (!quat.coeffs().allFinite() || quat.norm()<1e-8) {
      latest_.reset(); invalidate("Invalid odometry quaternion"); return;
    }
    const double stamp=seconds(m.header.stamp);
    if (!odoms_.empty() && stamp<=odoms_.back().stamp) {
      odoms_.clear(); frames_.clear(); pending_.clear(); passed_.clear(); heading_.reset(); last_cloud_=neg_inf;
      target_.reset();
      captured_z_.reset(); ready_=false;
      invalidate("Odometry time reset");
    }
    const Eigen::Matrix3d r=quat.normalized().toRotationMatrix();
    const Vec3 p=vector(m.pose.pose.position);
    Pose pose{stamp,p+r*body_offset_,p+r*lidar_offset_,r,vector(m.twist.twist.linear).norm()};
    if (!odoms_.empty()) pose.speed=std::max(pose.speed,(pose.body-odoms_.back().body).norm()/std::max(stamp-odoms_.back().stamp,1e-6));
    if (!pose.body.allFinite() || !pose.origin.allFinite() || !std::isfinite(pose.speed)) {
      latest_.reset(); invalidate("Nonfinite odometry"); return;
    }
    latest_=pose; odoms_.push_back(pose); if (odoms_.size()>200) odoms_.pop_front();
  }
  bool ingest() {
    bool fresh=false;
    while (!pending_.empty() && !odoms_.empty()) {
      const auto m=pending_.front(); const double stamp=seconds(m->header.stamp);
      const auto closest=std::min_element(odoms_.begin(),odoms_.end(),[stamp](const Pose &a,const Pose &b) {
        return std::abs(a.stamp-stamp)<std::abs(b.stamp-stamp);
      });
      if (std::abs(closest->stamp-stamp)>sync_tolerance_) {
        if (stamp>odoms_.back().stamp && now().seconds()-stamp<max_age_) break;
        pending_.pop_front(); continue;
      }
      pending_.pop_front(); if (stamp<=last_cloud_) continue;
      CloudView view; view.data=m->data.data(); view.size=m->data.size(); view.width=m->width; view.height=m->height;
      view.point_step=m->point_step; view.row_step=m->row_step; view.big_endian=m->is_bigendian;
      const std::array<std::string,3> names{"x","y","z"};
      for (size_t i=0; i<3; ++i) {
        const auto f=std::find_if(m->fields.begin(),m->fields.end(),[&](const auto &field) { return field.name==names[i]; });
        if (f==m->fields.end() || f->count!=1) throw std::invalid_argument("Missing scalar XYZ cloud field");
        view.xyz[i]={f->offset,f->datatype};
      }
      frames_.push_back({stamp,read_cloud(view,closest->body,self_radius_),closest->origin});
      last_cloud_=stamp; fresh=true;
    }
    return fresh;
  }
  void tick() {
    const double t=now().seconds();
    segment_clear_=false;
    if (!preview_ && auto_z_ && !(ready_ && t>=ready_stamp_ && t-ready_stamp_<=ready_timeout_)) {
      captured_z_.reset(); capture_start_=neg_inf;
    }
    if (!latest_ || t<latest_->stamp || t-latest_->stamp>max_age_) { invalidate("Waiting for fresh odometry"); return; }
    bool fresh;
    try { fresh=ingest(); }
    catch (const std::exception &e) { frames_.clear(); invalidate(e.what()); return; }
    while (!frames_.empty() && t-frames_.front().stamp>cache_seconds_) frames_.pop_front();
    if (frames_.empty() || t<frames_.back().stamp || t-frames_.back().stamp>max_age_) {
      invalidate("Waiting for synchronized fresh registered cloud"); return;
    }
    const auto &pose=*latest_; const Vec3 &body=pose.body;
    if (!preview_ && require_ready_ && !(ready_ && t>=ready_stamp_ && t-ready_stamp_<=ready_timeout_)) {
      if (active_) invalidate("Controller handoff missing or stale");
      else { clear_paths(); status("WAIT_CONTROL","Waiting for existing controller handoff"); }
      return;
    }
    if (!preview_ && auto_z_ && !captured_z_) {
      if (pose.speed>start_speed_) capture_start_=neg_inf;
      else if (!std::isfinite(capture_start_) || std::abs(body.z()-capture_z_)>height_tolerance_) {
        capture_start_=pose.stamp; capture_z_=body.z();
      } else if (pose.stamp-capture_start_>=1.0) {
        captured_z_=body.z();
        log_->write("height_captured",{{"flight_z",*captured_z_},{"odom_stamp",pose.stamp},{"frame_id",frame_}});
        RCLCPP_INFO(get_logger(),"Locked flight_z=%.3f m in %s",*captured_z_,frame_.c_str());
      }
      if (!captured_z_) {
        clear_paths(); status("WAIT_HEIGHT","Waiting for 1 s of stationary fresh LIO after handoff"); return;
      }
    }
    const double z=preview_ && follow_height_ ? body.z() :
        (!preview_ && auto_z_ ? *captured_z_ : flight_z_);
    if (std::abs(body.z()-z)>height_tolerance_) { invalidate("Vehicle deviated from locked/configured flight_z"); return; }
    const double yaw=std::atan2(pose.rotation(1,0),pose.rotation(0,0));
    if (!heading_) heading_=yaw;
    Eigen::Matrix2d r=basis(*heading_);
    const double slice_z=z+pose.origin.z()-body.z();
    if (!target_ && !active_ && count_==0) {
      Route slice;
      for (const auto &f : frames_) for (const auto &p : f.points) {
        const Vec2 local=r.transpose()*(p.head<2>()-body.head<2>());
        if (local.x()>=0.0 && std::abs(p.z()-slice_z)<slice_half_) slice.push_back(local);
      }
      if (!slice.empty()) { *heading_+=corridor_heading(slice); r=basis(*heading_); }
    }
    Grid detection(cfg_),collision(cfg_);
    // The rear buffer is outside the perception half-plane. Mark it ignored
    // instead of unknown so unknown inflation cannot block the start cell.
    const int zero_column=detection.cell(Vec2::Zero()).x;
    if (zero_column>0) {
      detection.observed.colRange(0,zero_column).setTo(1);
      collision.observed.colRange(0,zero_column).setTo(1);
    }
    for (const auto &f : frames_) {
      const Vec2 origin=r.transpose()*(f.origin.head<2>()-body.head<2>());
      Route observed_returns;
      for (const auto &p : f.points) {
        const Vec2 local=r.transpose()*(p.head<2>()-body.head<2>());
        if (local.x()<0.0) continue;
        if (std::abs(p.z()-slice_z)<slice_half_) {
          detection.add(local,origin);
          // Only near-horizontal rays establish observed free space.
          if (std::abs(f.origin.z()-slice_z)<slice_half_) observed_returns.push_back(local);
        }
        if (p.z()>=z-below_-vertical_margin_ && p.z()<=z+above_+vertical_margin_) collision.occupy(local);
      }
      observe_scan(collision,observed_returns,origin);
    }
    complete_observation(collision);
    // The space currently occupied by the vehicle is a known starting footprint,
    // even when self-filtering removes returns. Never erase measured obstacles.
    for (int y=0; y<collision.observed.rows; ++y) for (int x=0; x<collision.observed.cols; ++x)
      if (collision.point(x,y).norm()<=cfg_.radius) collision.observed(y,x)=1;
    sensor_msgs::msg::Image image; image.header=header(*this,frame_); image.encoding="mono8";
    image.height=detection.occupied.rows; image.width=detection.occupied.cols; image.step=image.width;
    image.data.reserve(detection.occupied.total());
    for (int y=0; y<detection.occupied.rows; ++y) for (int x=0; x<detection.occupied.cols; ++x)
      image.data.push_back(detection.occupied(y,x)*255);
    image_pub_->publish(image);
    const auto layers=collision.inflation_layers();
     
    segment_clear_=false;
    if (passed_.size()>=2) { clear_paths(); status("COMPLETE","Completed two door crossings"); return; }
    if (!target_) {
      if (!fresh) return;
      auto gates=detect_gates(detection);
      gates.erase(std::remove_if(gates.begin(),gates.end(),[&](const Gate &g) {
        const Vec2 center=world_xy(g.center,body,*heading_);
        return std::any_of(passed_.begin(),passed_.end(),[&](const Vec2 &p) { return (center-p).norm()<=0.5; });
      }),gates.end());
      
      if (gates.empty()) { plan_stage_.clear(); candidate_.reset(); count_=0; clear_paths(); status("SEARCH","No confirmed opening"); return; }
      const auto &gate=gates.front(); const Vec2 center=world_xy(gate.center,body,*heading_);
      count_=candidate_ && (center-*candidate_).norm()<0.06 ? count_+1 : 1; candidate_=center;
      const Vec2 normal=r*gate.normal.normalized();
      std_msgs::msg::String found;
      found.data=Json{{"frame_id",frame_},{"stamp",t},{"center",{center.x(),center.y(),z}},
        {"normal",{normal.x(),normal.y(),0.0}},{"width",gate.width},{"stable_observations",count_},
        {"height_source",preview_ && follow_height_?"current_odometry":
            (!preview_ && auto_z_?"locked_odometry":"configured")},{"height_verified",false}}.dump();
      detection_pub_->publish(found); log_->write("detection",Json::parse(found.data));
      if (count_<stable_frames_) { clear_paths(); status("VERIFY"); return; }
      if (fixed_gate_route(cfg_,gate).empty()) {
        clear_paths(); status("NO_PATH","Door approach point is behind vehicle; reverse motion is disabled"); return;
      }
      target_=Target{Gate{center,normal,gate.width,
        {world_xy(gate.edges[0],body,*heading_),world_xy(gate.edges[1],body,*heading_)}}};
      log_->write("target_locked",{{"door_index",passed_.size()+1},{"center",{center.x(),center.y()}},
        {"normal",{normal.x(),normal.y()}},{"width",gate.width}});
      RCLCPP_INFO(get_logger(),"Door %zu locked: center=(%.3f, %.3f), approach=%.2f m, crossing=%.2f m",
        passed_.size()+1,center.x(),center.y(),cfg_.pre_distance,cfg_.pre_distance+cfg_.post_distance);
    }
    const auto &world=target_->gate;
    const Vec2 pre=world.center-cfg_.pre_distance*world.normal;
    const Vec2 post=world.center+cfg_.post_distance*world.normal;
    bool advanced=false;
    const Vec2 goal=target_->crossing?post:pre;
    if ((body.head<2>()-goal).norm()<0.08 && pose.speed<start_speed_) {
      stop_motion();
      if (target_->crossing) {
        passed_.push_back(world.center); target_.reset(); candidate_.reset(); count_=0;
        clear_paths();  plan_stage_.clear();
        log_->write("door_passed",{{"passed_count",passed_.size()}});
        status(passed_.size()==2?"COMPLETE":"PASSED"); return;
      }
      target_->crossing=true; advanced=true;
      log_->write("phase_change",{{"door_index",passed_.size()+1},{"plan_stage","CROSS"}});
    }
    plan_stage_=target_->crossing?"CROSS":"APPROACH";
    
    const Route guide=target_->crossing?Route{body.head<2>(),post}:Route{body.head<2>(),pre,post};
    publish_geometry(guide,z,yaw);
    const Vec2 endpoint=target_->crossing?post:pre;
    const Vec2 local_goal=r.transpose()*(endpoint-body.head<2>());
    const std::string reason=local_goal.x() < -cfg_.resolution ? "Goal behind vehicle; reverse motion is disabled" :
      straight_segment_check(collision,layers,Vec2::Zero(),local_goal);
    segment_clear_=reason.empty();
    if (fresh) {
      auto cell_state=[&](const Vec2 &p) {
        const auto q=collision.cell(p);
        if (!collision.inside(q)) return Json{{"inside",false}};
        return Json{{"inside",true},{"obstacle",layers.obstacles(q)!=0},
          {"unknown",layers.unknown(q)!=0},{"observed",collision.observed(q)!=0}};
      };
      log_->write("planning_check",{{"start",cell_state(Vec2::Zero())},{"gate",cell_state(r.transpose()*(world.center-body.head<2>()))},
        {"pre",cell_state(r.transpose()*(pre-body.head<2>()))},
        {"post",cell_state(r.transpose()*(post-body.head<2>()))},
        {"reason",reason},{"crossing",target_->crossing},{"segment_clear",segment_clear_}});
    }
    if (advanced) { status("OBSERVE","At approach point; checking the crossing segment"); return; }
    if (active_) {
      if (!segment_clear_) { invalidate(reason); return; }
      const double elapsed=t-active_->start;
      const auto &samples=active_->samples;
      if (elapsed<0 || elapsed>samples.back().t+0.5) { invalidate("Trajectory expired or time reset"); return; }
      Vec3 expected=samples.back().p;
      const auto next=std::upper_bound(samples.begin(),samples.end(),elapsed,[](double v,const Sample &s) { return v<s.t; });
      if (next!=samples.end()) expected=next==samples.begin()?next->p:interpolate(*(next-1),*next,elapsed).p;
      if ((body.head<2>()-expected.head<2>()).norm()>observation_tolerance_) { invalidate("Execution deviated from trajectory"); return; }
      Vec2 previous=Vec2::Zero();
      for (const auto &s : samples) if (s.t>=elapsed) {
        const Vec2 p=r.transpose()*(s.p.head<2>()-body.head<2>());
        if (p.x() < -cfg_.resolution) { invalidate("Remaining trajectory leaves forward perception"); return; }
        if (!clear_segment(collision,layers.blocked,previous,p)) { invalidate("Remaining trajectory blocked or unobserved"); return; }
        previous=p;
      }
      status("EXECUTING","",true); return;
    }
    if (preview_) { status("PREVIEW",segment_clear_?"":"Geometric route only: "+reason); return; }
    if (!segment_clear_) { status("WAIT_VIEW",reason); return; }
    if (pose.speed>start_speed_) { status("WAIT_FOR_REST","Each segment starts at rest"); return; }
    if (!fresh) return;
    auto samples=trajectory({body.head<2>(),endpoint},cfg_);
    for (auto &s : samples) s.p.z()=z;
    if (samples.empty()) { status("WAIT_FOR_REST","Waiting for measured arrival"); return; }
    const double start=publish_plan(samples,z,yaw);
    active_=Active{std::move(samples),start}; status("READY","",true);
  }
  void stop_motion() {
    if (active_) { Motion empty; empty.header=header(*this,frame_); motion_pub_->publish(empty); }
    active_.reset();
  }
  void publish_geometry(const Route &route,double z,double yaw) {
    path_pub_->publish(make_path(route,z,yaw,header(*this,frame_)));
    
    Json points=Json::array();
    for (const auto &p : route) points.push_back({p.x(),p.y(),z});
    log_->write("nominal_path",{{"plan_stage",plan_stage_},{"door_index",passed_.size()+1},
      {"path_kind","geometric_reference"},{"points",points}});
  }
  double publish_plan(const Trajectory &samples,double z,double yaw) {
    const auto h=header(*this,frame_);
    if (log_->enabled()) {
      Json points=Json::array();
      for (const auto &s : samples) points.push_back({{"t",s.t},{"p",{s.p.x(),s.p.y(),s.p.z()}},
        {"v",{s.v.x(),s.v.y(),s.v.z()}},{"a",{s.a.x(),s.a.y(),s.a.z()}}});
      log_->write("plan",{{"start_stamp",seconds(h.stamp)},{"frame_id",frame_},
        {"preview",preview_},{"plan_stage",plan_stage_},{"flight_z",z},{"yaw",yaw},{"samples",points}});
    }
    Motion motion; motion.header=h; motion.joint_names={"base_link"}; motion.points.reserve(samples.size());
    for (const auto &s : samples) {
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint p;
      geometry_msgs::msg::Transform tf; assign(tf.translation,s.p); tf.rotation=quaternion(yaw);
      geometry_msgs::msg::Twist v,a; assign(v.linear,s.v); assign(a.linear,s.a);
      p.transforms.push_back(tf); p.velocities.push_back(v); p.accelerations.push_back(a);
      p.time_from_start=rclcpp::Duration::from_seconds(s.t); motion.points.push_back(std::move(p));
    }
    motion_pub_->publish(motion); return seconds(h.stamp);
  }
};
}  // namespace door_navigation

#ifndef DOOR_NODE_NO_MAIN
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<door_navigation::DoorNode>()); }
  catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("door_navigation"),"%s",e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
#endif
