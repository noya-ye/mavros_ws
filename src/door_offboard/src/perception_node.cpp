#include "door_navigation/cloud.hpp"
#include "door_navigation/ros_helpers.hpp"
#include "door_navigation/diagnostic_log.hpp"
#include <door_offboard/msg/planner_enable.hpp>
#include <door_offboard/msg/door_goal.hpp>
#include <door_offboard/msg/goal_reached.hpp>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <chrono>
#include <deque>

namespace door_navigation {
using Json=nlohmann::json;
using Enable=door_offboard::msg::PlannerEnable;
using Goal=door_offboard::msg::DoorGoal;
using Reached=door_offboard::msg::GoalReached;
using Odom=nav_msgs::msg::Odometry;
using Cloud=sensor_msgs::msg::PointCloud2;

class PerceptionNode : public rclcpp::Node {
 public:
  PerceptionNode() : Node("door_perception") {
    frame_=declare_parameter("planning_frame",std::string("lidar"));
    child_=declare_parameter("odom_child_frame",std::string("body"));
    const auto cloud=declare_parameter("cloud_topic",std::string("/fastlio2/world_cloud"));
    const auto odom=declare_parameter("odom_topic",std::string("/fastlio2/lio_odom"));
    cfg_.radius=number("radius",0.25); cfg_.margin=number("margin",0.0,true);
    cfg_.x_min=-(cfg_.radius+cfg_.margin+cfg_.resolution);
    cfg_.corridor_width=number("corridor_width",2.2);
    cfg_.corridor_tolerance=number("corridor_tolerance",0.5);
    cfg_.door_min=number("door_min",0.5); cfg_.door_max=number("door_max",1.2);
    cfg_.y_max=number("lateral_range",2.0); cfg_.y_min=-cfg_.y_max;
    cfg_.pre_distance=number("pre_distance",0.5); cfg_.post_distance=number("post_distance",0.5);
    cfg_.wall_thickness=number("wall_thickness",0.15,true); cfg_.validate();
    below_=number("body_below",0.15,true); above_=number("body_above",0.15,true);
    vertical_margin_=number("vertical_margin",0.08,true);
    slice_half_=number("slice_half_thickness",0.08); self_radius_=number("self_filter_radius",0.28,true);
    max_age_=number("max_age",0.3); cache_seconds_=number("cache_seconds",0.6);
    sync_=number("sync_tolerance",0.03); enable_timeout_=number("enable_timeout",0.5);
    height_tolerance_=number("height_tolerance",0.06);
    arrival_=number("arrival_tolerance",0.10); stop_speed_=number("stop_speed",0.05);
    stable_frames_=declare_parameter("stable_frames",3);
    max_doors_=declare_parameter("max_doors",2);
    if (stable_frames_<1 || max_doors_<1 || max_doors_>2) throw std::invalid_argument("Invalid stability/door count");
    lidar_offset_=offset("lidar_offset"); body_offset_=offset("body_offset");
    log_=std::make_unique<DiagnosticLog>(*this);
    goal_pub_=create_publisher<Goal>("/door/goal",10);
    status_pub_=create_publisher<std_msgs::msg::String>("/door/status",10);
    detection_pub_=create_publisher<std_msgs::msg::String>("/door/detection",10);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/door/path",10);
    image_pub_=create_publisher<sensor_msgs::msg::Image>("/door/debug_image",rclcpp::SensorDataQoS());
    enable_sub_=create_subscription<Enable>("/door/enable",10,[this](Enable::ConstSharedPtr m) {
      if (m->header.frame_id!=frame_ || !fresh(seconds(m->header.stamp),enable_timeout_) ||
          !std::isfinite(m->flight_z_lio) || m->session_id==0) return;
      if (m->session_id!=session_) {
        reset(); session_=m->session_id; flight_z_=m->flight_z_lio;
        log_->write("session",{{"session_id",session_},{"flight_z_lio",flight_z_},{"enabled",m->enabled}});
      }
      if (std::abs(m->flight_z_lio-flight_z_)>1e-6) { fatal_="Height changed within a session"; return; }
      enabled_=m->enabled; enable_stamp_=seconds(m->header.stamp);
    });
    reached_sub_=create_subscription<Reached>("/door/reached",10,[this](Reached::ConstSharedPtr m) {
      if (m->session_id==session_ && m->goal_id==goal_id_ && m->goal_id==last_valid_id_ &&
          m->header.frame_id==frame_ && fresh(seconds(m->header.stamp),enable_timeout_)) {
        if (ack_!=m->goal_id) log_->write("arrival_feedback",{{"session_id",session_},{"goal_id",m->goal_id}});
        ack_=m->goal_id;
      }
    });
    odom_sub_=create_subscription<Odom>(odom,rclcpp::SensorDataQoS(),[this](Odom::ConstSharedPtr m) { on_odom(*m); });
    cloud_sub_=create_subscription<Cloud>(cloud,rclcpp::SensorDataQoS(),[this](Cloud::ConstSharedPtr m) {
      if (!enabled_ || !fresh(enable_stamp_,enable_timeout_)) return;
      if (m->header.frame_id!=frame_) { fatal_="Cloud frame mismatch"; return; }
      pending_.push_back(m); if (pending_.size()>10) pending_.pop_front();
    });
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this] {
      try { tick(); } catch (const std::exception &e) { frames_.clear(); publish("INVALID",e.what()); }
    });
  }
 private:
  struct Pose { double stamp; Vec3 body,origin; Eigen::Matrix3d rotation; double speed; };
  struct Frame { double stamp; std::vector<Vec3> points; Vec3 origin; };
  struct Target { Gate gate; bool crossing=false; };
  Config cfg_;
  std::string frame_,child_,fatal_;
  double below_,above_,vertical_margin_,slice_half_,self_radius_,max_age_,cache_seconds_,sync_;
  double enable_timeout_,height_tolerance_,arrival_,stop_speed_,flight_z_=0;
  double last_cloud_=neg_inf,enable_stamp_=neg_inf;
  int stable_frames_,max_doors_,count_=0;
  bool enabled_=false;
  uint64_t session_=0,goal_id_=0,last_valid_id_=0,ack_=0;
  Vec3 lidar_offset_,body_offset_;
  std::deque<Pose> odoms_;
  std::deque<Frame> frames_;
  std::deque<Cloud::ConstSharedPtr> pending_;
  std::optional<Target> target_;
  std::optional<Vec2> candidate_;
  std::optional<double> heading_;
  Route passed_;
  std::unique_ptr<DiagnosticLog> log_;
  rclcpp::Publisher<Goal>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,detection_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<Enable>::SharedPtr enable_sub_;
  rclcpp::Subscription<Reached>::SharedPtr reached_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double number(const std::string &name,double value,bool zero=false) {
    const double v=declare_parameter(name,value);
    if (!std::isfinite(v) || (zero?v<0:v<=0)) throw std::invalid_argument(name);
    return v;
  }
  Vec3 offset(const std::string &name) {
    const auto v=declare_parameter(name,std::vector<double>{0,0,0});
    if (v.size()!=3) throw std::invalid_argument(name);
    Vec3 p(v[0],v[1],v[2]); if (!p.allFinite()) throw std::invalid_argument(name); return p;
  }
  bool fresh(double stamp,double age) const {
    const double t=now().seconds(); return t>=stamp && t-stamp<=age;
  }
  void reset() {
    frames_.clear(); pending_.clear(); target_.reset(); candidate_.reset(); heading_.reset(); passed_.clear();
    last_cloud_=neg_inf; count_=0; goal_id_=last_valid_id_=ack_=0; fatal_.clear();
  }
  Vec2 endpoint() const {
    const auto &g=target_->gate;
    return g.center+(target_->crossing?cfg_.post_distance:-cfg_.pre_distance)*g.normal;
  }
  void publish(const std::string &state,const std::string &reason="",bool valid=false) {
    Goal goal; goal.header=header(*this,frame_); goal.session_id=session_; goal.goal_id=goal_id_;
    goal.valid=valid; goal.complete=state=="COMPLETE"; goal.stage=target_?(target_->crossing?"CROSS":"APPROACH"):state;
    goal.reason=reason; if (target_) goal.target=point(endpoint(),flight_z_);
    if (valid) last_valid_id_=goal_id_;
    goal_pub_->publish(goal);
    Json data={{"state",state},{"reason",reason},{"session_id",session_},{"goal_id",goal_id_},
      {"plan_stage",goal.stage},{"segment_clear",valid},{"passed_count",passed_.size()},
      {"target_locked",target_.has_value()},{"flight_z_lio",flight_z_},{"stamp",now().seconds()}};
    const double t=now().seconds();
    data["frame_id"]=frame_; data["stable_observations"]=count_;
    data["cached_frames"]=frames_.size(); data["pending_clouds"]=pending_.size();
    data["enable_age_s"]=std::isfinite(enable_stamp_)?Json(t-enable_stamp_):Json(nullptr);
    data["cloud_age_s"]=std::isfinite(last_cloud_)?Json(t-last_cloud_):Json(nullptr);
    if (!odoms_.empty()) {
      data["position"]={odoms_.back().body.x(),odoms_.back().body.y(),odoms_.back().body.z()};
      data["odom_age_s"]=t-odoms_.back().stamp; data["speed_mps"]=odoms_.back().speed;
      data["height_error_m"]=odoms_.back().body.z()-flight_z_;
    }
    if (target_) data["target"]={goal.target.x,goal.target.y,goal.target.z};
    std_msgs::msg::String msg; msg.data=data.dump(); status_pub_->publish(msg); log_->write("status",data);
    if (state=="INVALID" || state=="DISABLED" || state=="COMPLETE" || !target_)
      path_pub_->publish(make_path({},flight_z_,0,header(*this,frame_)));
  }
  void on_odom(const Odom &m) {
    if (m.header.frame_id!=frame_ || m.child_frame_id!=child_) { fatal_="Odometry frame mismatch"; return; }
    const auto &q=m.pose.pose.orientation; Eigen::Quaterniond quat(q.w,q.x,q.y,q.z);
    if (!quat.coeffs().allFinite() || quat.norm()<1e-8) { fatal_="Invalid odometry quaternion"; return; }
    const double stamp=seconds(m.header.stamp);
    if (!odoms_.empty() && stamp<=odoms_.back().stamp) {
      if (stamp==odoms_.back().stamp) return;
      fatal_="LIO time reset; restart flight session"; odoms_.clear(); frames_.clear(); return;
    }
    const auto r=quat.normalized().toRotationMatrix(); const Vec3 p=vector(m.pose.pose.position);
    Pose pose{stamp,p+r*body_offset_,p+r*lidar_offset_,r,vector(m.twist.twist.linear).norm()};
    if (!odoms_.empty()) pose.speed=std::max(pose.speed,(pose.body-odoms_.back().body).norm()/(stamp-odoms_.back().stamp));
    if (!pose.body.allFinite() || !pose.origin.allFinite() || !std::isfinite(pose.speed)) { fatal_="Nonfinite odometry"; return; }
    odoms_.push_back(pose); if (odoms_.size()>200) odoms_.pop_front();
  }
  bool ingest() {
    bool updated=false;
    while (!pending_.empty() && !odoms_.empty()) {
      const auto m=pending_.front(); const double stamp=seconds(m->header.stamp);
      auto closest=std::min_element(odoms_.begin(),odoms_.end(),[stamp](const Pose &a,const Pose &b) {
        return std::abs(a.stamp-stamp)<std::abs(b.stamp-stamp);
      });
      if (std::abs(closest->stamp-stamp)>sync_) {
        if (stamp>odoms_.back().stamp && fresh(stamp,max_age_)) break;
        log_->write("cloud_dropped",{{"reason","No synchronized odometry"},{"cloud_stamp",stamp},
          {"odom_stamp",closest->stamp},{"sync_error_s",closest->stamp-stamp},{"session_id",session_}});
        pending_.pop_front(); continue;
      }
      pending_.pop_front(); if (stamp<=last_cloud_) continue;
      CloudView v; v.data=m->data.data(); v.size=m->data.size(); v.width=m->width; v.height=m->height;
      v.point_step=m->point_step; v.row_step=m->row_step; v.big_endian=m->is_bigendian;
      for (size_t i=0;i<3;++i) {
        const auto name=std::array<std::string,3>{"x","y","z"}[i];
        const auto f=std::find_if(m->fields.begin(),m->fields.end(),[&](const auto &field) { return field.name==name; });
        if (f==m->fields.end() || f->count!=1) throw std::runtime_error("Missing XYZ cloud field");
        v.xyz[i]={f->offset,f->datatype};
      }
      frames_.push_back({stamp,read_cloud(v,closest->body,self_radius_),closest->origin});
      log_->write("cloud",{{"session_id",session_},{"cloud_stamp",stamp},{"odom_stamp",closest->stamp},
        {"sync_error_s",closest->stamp-stamp},{"frame_id",m->header.frame_id},
        {"input_points",static_cast<uint64_t>(m->width)*m->height},
        {"filtered_points",frames_.back().points.size()}});
      last_cloud_=stamp; updated=true;
    }
    return updated;
  }
  void tick() {
    if (!enabled_ || !fresh(enable_stamp_,enable_timeout_)) {
      frames_.clear(); pending_.clear(); publish("DISABLED","Waiting for stable takeoff handoff"); return;
    }
    if (!fatal_.empty()) { publish("INVALID",fatal_); return; }
    if (odoms_.empty() || !fresh(odoms_.back().stamp,max_age_)) { publish("INVALID","Stale odometry"); return; }
    const bool updated=ingest();
    while (!frames_.empty() && now().seconds()-frames_.front().stamp>cache_seconds_) frames_.pop_front();
    if (frames_.empty() || !fresh(frames_.back().stamp,max_age_)) { publish("INVALID","Stale synchronized cloud"); return; }
    const auto &pose=odoms_.back(); const Vec3 &body=pose.body;
    if (std::abs(body.z()-flight_z_)>height_tolerance_) { publish("INVALID","Outside fixed planning height"); return; }
    if (passed_.size()>=static_cast<size_t>(max_doors_)) { publish("COMPLETE"); return; }
    if (target_ && ack_==goal_id_ && (body.head<2>()-endpoint()).norm()<=arrival_ && pose.speed<=stop_speed_) {
      log_->write("stage_reached",{{"session_id",session_},{"goal_id",goal_id_},
        {"stage",target_->crossing?"CROSS":"APPROACH"},{"position",{body.x(),body.y(),body.z()}},
        {"xy_error_m",(body.head<2>()-endpoint()).norm()},{"speed_mps",pose.speed}});
      ack_=0;
      if (target_->crossing) { passed_.push_back(target_->gate.center); target_.reset(); candidate_.reset(); count_=0;
        publish(passed_.size()>=static_cast<size_t>(max_doors_)?"COMPLETE":"SEARCH"); return; }
      target_->crossing=true; ++goal_id_;
    }
    if (!heading_) heading_=std::atan2(pose.rotation(1,0),pose.rotation(0,0));
    auto r=basis(*heading_); const double slice_z=flight_z_+pose.origin.z()-body.z();
    if (!target_ && count_==0) {
      Route slice;
      for (const auto &f:frames_) for (const auto &p:f.points) {
        Vec2 local=r.transpose()*(p.head<2>()-body.head<2>());
        if (local.x()>=0 && std::abs(p.z()-slice_z)<slice_half_) slice.push_back(local);
      }
      if (!slice.empty()) { *heading_+=corridor_heading(slice); r=basis(*heading_); }
    }
    Grid detection(cfg_),collision(cfg_);
    const int zero=detection.cell(Vec2::Zero()).x;
    if (zero>0) { detection.observed.colRange(0,zero).setTo(1); collision.observed.colRange(0,zero).setTo(1); }
    size_t ray_count=0;
    for (const auto &f:frames_) {
      const Vec2 origin=r.transpose()*(f.origin.head<2>()-body.head<2>()); Route returns;
      for (const auto &p:f.points) {
        const Vec2 local=r.transpose()*(p.head<2>()-body.head<2>()); if (local.x()<0) continue;
        if (std::abs(p.z()-slice_z)<slice_half_) {
          detection.add(local,origin); if (std::abs(f.origin.z()-slice_z)<slice_half_) returns.push_back(local);
        }
        if (p.z()>=flight_z_-below_-vertical_margin_ && p.z()<=flight_z_+above_+vertical_margin_) collision.occupy(local);
      }
      ray_count+=returns.size();
      observe_scan(collision,returns,origin);
    }
    complete_observation(collision);
    const int inferred=complete_corridor_observation(collision,
      r.transpose()*(frames_.back().origin.head<2>()-body.head<2>()));
    for (int y=0;y<collision.observed.rows;++y) for (int x=0;x<collision.observed.cols;++x)
      if (collision.point(x,y).norm()<=cfg_.radius) collision.observed(y,x)=1;
    log_->write("scene",{{"session_id",session_},{"heading",*heading_},{"slice_z",slice_z},
      {"rays",ray_count},{"frames",frames_.size()},
      {"slice_obstacle_cells",cv::countNonZero(detection.occupied)},
      {"body_obstacle_cells",cv::countNonZero(collision.occupied)},
      {"corridor_model_active",inferred>=0},{"inferred_free_cells",std::max(0,inferred)},
      {"unknown_blocks_path",false},
      {"observed_cells",cv::countNonZero(collision.observed)},
      {"total_cells",collision.observed.total()}});
    sensor_msgs::msg::Image image; image.header=header(*this,frame_); image.encoding="mono8";
    image.height=detection.occupied.rows; image.width=detection.occupied.cols; image.step=image.width;
    for (int y=0;y<detection.occupied.rows;++y) for (int x=0;x<detection.occupied.cols;++x)
      image.data.push_back(detection.occupied(y,x)*255);
    image_pub_->publish(image);
    if (!target_) {
      if (!updated) { publish("SEARCH"); return; }
      auto gates=detect_gates(detection);
      gates.erase(std::remove_if(gates.begin(),gates.end(),[&](const Gate &g) {
        Vec2 c=world_xy(g.center,body,*heading_);
        return std::any_of(passed_.begin(),passed_.end(),[&](const Vec2 &p) { return (c-p).norm()<=0.5; });
      }),gates.end());
      if (gates.empty()) { candidate_.reset(); count_=0; publish("SEARCH","No opening"); return; }
      const auto &g=gates.front(); const Vec2 c=world_xy(g.center,body,*heading_);
      count_=candidate_ && (c-*candidate_).norm()<0.06?count_+1:1; candidate_=c;
      const Vec2 n=r*g.normal.normalized();
      Json data={{"center",{c.x(),c.y(),flight_z_}},{"normal",{n.x(),n.y(),0}},
        {"width",g.width},{"stable_observations",count_},{"frame_id",frame_}};
      std_msgs::msg::String found; found.data=data.dump(); detection_pub_->publish(found); log_->write("detection",data);
      if (count_<stable_frames_) { publish("VERIFY"); return; }
      if (fixed_gate_route(cfg_,g).empty()) { publish("WAIT_VIEW","Door prepoint behind aircraft"); return; }
      target_=Target{Gate{c,n,g.width,{world_xy(g.edges[0],body,*heading_),world_xy(g.edges[1],body,*heading_)}}}; ++goal_id_;
      log_->write("door_locked",{{"session_id",session_},{"goal_id",goal_id_},{"door_number",passed_.size()+1},
        {"center",{c.x(),c.y(),flight_z_}},{"normal",{n.x(),n.y()}},{"width",g.width}});
      RCLCPP_INFO(get_logger(),"Locked door %zu: (%.3f, %.3f)",passed_.size()+1,c.x(),c.y());
    }
    const auto &g=target_->gate; const Vec2 pre=g.center-cfg_.pre_distance*g.normal,post=g.center+cfg_.post_distance*g.normal;
    const Route guide=target_->crossing?Route{body.head<2>(),post}:Route{body.head<2>(),pre,post};
    path_pub_->publish(make_path(guide,flight_z_,*heading_,header(*this,frame_)));
    const Vec2 local_goal=r.transpose()*(endpoint()-body.head<2>());
    const auto layers=collision.inflation_layers();
    const std::string reason=local_goal.x() < -cfg_.resolution?"Goal behind forward perception":
      straight_segment_check(collision,layers,Vec2::Zero(),local_goal);
    if (log_->enabled()) {
      const auto start=collision.cell(Vec2::Zero()); Json nearby=Json::array();
      if (layers.unknown(start)) for (int y=0;y<collision.observed.rows;++y) for (int x=0;x<collision.observed.cols;++x)
        if (!collision.observed(y,x) && std::hypot(x-start.x,y-start.y)*cfg_.resolution<=cfg_.radius+cfg_.margin) {
          const auto p=collision.point(x,y); nearby.push_back({p.x(),p.y()});
        }
      log_->write("planning_check",{{"reason",reason},{"heading",*heading_},
        {"session_id",session_},{"goal_id",goal_id_},{"stage",target_->crossing?"CROSS":"APPROACH"},
        {"path",target_->crossing?Json{{body.x(),body.y(),flight_z_},{post.x(),post.y(),flight_z_}}:
          Json{{body.x(),body.y(),flight_z_},{pre.x(),pre.y(),flight_z_},{post.x(),post.y(),flight_z_}}},
        {"goal_local",{local_goal.x(),local_goal.y()}},{"unknown_near_start",nearby},
        {"rays",ray_count},{"frames",frames_.size()},
        {"scan_origin",{frames_.back().origin.x(),frames_.back().origin.y(),frames_.back().origin.z()}},
        {"slice_z",slice_z},{"points",frames_.back().points.size()}});
    }
    publish(reason.empty()?"READY":"WAIT_VIEW",reason,reason.empty());
  }
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<door_navigation::PerceptionNode>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("door_perception"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
