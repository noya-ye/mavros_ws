#pragma once
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace door_navigation {
// JSON Lines, one file per process run. Flush at least once per second.
class DiagnosticLog {
 public:
  explicit DiagnosticLog(rclcpp::Node &node) : node_(node) {
    const bool enabled=node.declare_parameter("enable_logging",true);
    std::string directory=node.declare_parameter("log_directory",std::string());
    if (!enabled) return;
    try {
      if (directory.empty()) {
        const char *ros_home=std::getenv("ROS_HOME"),*home=std::getenv("HOME");
        directory=(ros_home ? std::filesystem::path(ros_home) :
          (home ? std::filesystem::path(home)/".ros" : std::filesystem::temp_directory_path()))/"door_offboard";
      }
      std::filesystem::create_directories(directory);
      const auto stamp=std::chrono::system_clock::now().time_since_epoch();
      const auto path=std::filesystem::path(directory)/(std::string(node.get_name())+"_"+
        std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count())+".jsonl");
      file_.open(path,std::ios::out);
      if (!file_) throw std::runtime_error("Cannot open log file");
      RCLCPP_INFO(node.get_logger(),"Diagnostic log: %s",path.c_str());
      nlohmann::json config=nlohmann::json::object();
      for (const auto &name : node.list_parameters({},1).names)
        config[name]=node.get_parameter(name).value_to_string();
      write("startup",{{"schema_version",2},{"parameters",config}});
    } catch (const std::exception &e) {
      file_.close(); RCLCPP_WARN(node.get_logger(),"Diagnostic logging disabled: %s",e.what());
    }
  }
  ~DiagnosticLog() {
    write("shutdown",{{"reason","Node destroyed normally"}});
  }
  bool enabled() const { return file_.is_open(); }
  void write(const std::string &event,const nlohmann::json &data) {
    if (!file_.is_open()) return;
    try {
      const auto steady=std::chrono::steady_clock::now();
      const double wall=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      file_<<nlohmann::json{{"event",event},{"stamp",node_.now().seconds()},{"wall_stamp",wall},
        {"elapsed_s",std::chrono::duration<double>(steady-started_).count()},
        {"node",node_.get_name()},{"data",data}}.dump()<<'\n';
      if ((event!="status" && event!="planning_check" && event!="detection" && event!="scene" && event!="cloud") ||
          steady-last_flush_>=std::chrono::seconds(1)) {
        file_.flush(); last_flush_=steady;
      }
      if (!file_) throw std::runtime_error("Log write or flush failed");
    } catch (const std::exception &e) {
      file_.close(); RCLCPP_WARN(node_.get_logger(),"Diagnostic logging disabled: %s",e.what());
    }
  }
 private:
  rclcpp::Node &node_;
  std::ofstream file_;
  std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_flush_{};
};
}  // namespace door_navigation
