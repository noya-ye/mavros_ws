#include "offboard_core_pkg/mavros_iface.hpp"

#include <cmath>
#include <utility>

namespace offboard_core_pkg {
namespace {
constexpr char kStateTopic[] = "/mavros/state";
constexpr char kPoseTopic[] = "/mavros/local_position/pose";
constexpr char kVelocityTopic[] = "/mavros/local_position/velocity_local";
constexpr char kPositionSetpointTopic[] = "/mavros/setpoint_position/local";
}  // namespace

MavrosIface::MavrosIface(rclcpp::Node &node, Context &context) : node_(node), ctx_(context) {
  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
    kStateTopic, rclcpp::QoS(10), [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
      ctx_.connected = msg->connected;
      ctx_.armed = msg->armed;
      ctx_.mode = msg->mode;
    });
    // 订阅飞机状态，写入cotext
  pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    kPoseTopic, rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      ctx_.position_enu = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
      const auto &q = msg->pose.orientation;
      ctx_.yaw_enu = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      ctx_.position_valid = ctx_.finitePosition();
      if (!ctx_.home_initialized && ctx_.position_valid) {
        ctx_.home_initialized = true;
        ctx_.home_enu = ctx_.position_enu;
        ctx_.home_yaw_enu = ctx_.yaw_enu;
        ctx_.position_setpoint_enu = ctx_.home_enu;
        ctx_.yaw_setpoint_enu = ctx_.home_yaw_enu;
      }
    });
    // 订阅当前飞机位置信息，写入context，并在第一次获取到有效位置时初始化home点
  velocity_sub_ = node_.create_subscription<geometry_msgs::msg::TwistStamped>(
    kVelocityTopic, rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
      ctx_.velocity_enu = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
    });
    // 订阅当前飞机速度信息，写入context

    
  position_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    kPositionSetpointTopic, rclcpp::QoS(10));
    // 发布setpoint：控制飞机的目标位置和姿态，使飞机保持offboard


    // ServiceType
    // ↓
    // mavros_msgs::srv::SetMode

    // Service Name
    //     ↓
    // /mavros/set_mode
    // 此处为client的创建：
  arming_client_ = node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  // 用于发送arming/disarming请求，并接受响应。常见请求有：arming、disarming
  mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  // 用于发送模式切换请求，并接受响应。常见模式有：OFFBOARD、POSCTL、ALTCTL、MANUAL等
  land_client_ = node_.create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
  // 用于发送降落请求，并接受响应。常见请求有：land、takeoff
}

void MavrosIface::publishSetpoint() {
  if (!ctx_.publish_position_setpoint) return;
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = "map";
  msg.pose.position.x = ctx_.position_setpoint_enu.x;
  msg.pose.position.y = ctx_.position_setpoint_enu.y;
  msg.pose.position.z = ctx_.position_setpoint_enu.z;
  msg.pose.orientation.z = std::sin(ctx_.yaw_setpoint_enu / 2.0);
  msg.pose.orientation.w = std::cos(ctx_.yaw_setpoint_enu / 2.0);
  position_pub_->publish(msg);
}
// 发布setpoint：控制飞机的目标位置和姿态，使飞机保持offboard。这个函数只发布一次，不循环发布，在主节点中调用定时器进行循环发布


bool MavrosIface::requestMode(const std::string &mode, CommandCallback callback) {
  if (!mode_client_->service_is_ready()) return false;
  // 当前是否已经发现：/mavros/set_mode这个 Service Server？

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  mode_client_->async_send_request(request, [callback = std::move(callback)](
      rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    if (callback) callback(future.get()->mode_sent);
  });

  // 发送request,通过async_send_request立刻获得是否成功发射出去
  // 然后通过回调函数callback获取服务端的响应结果，future.get()返回一个SetMode::Response对象，里面包含mode_sent字段，表示是否成功切换模式。
  return true;
}

bool MavrosIface::requestArm(bool arm, CommandCallback callback) {
  if (!arming_client_->service_is_ready()) return false;
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;
  arming_client_->async_send_request(request, [callback = std::move(callback)](
      rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    if (callback) callback(future.get()->success);
  });
  return true;
}

bool MavrosIface::requestLand(CommandCallback callback) {
  if (!land_client_->service_is_ready()) return false;
  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  land_client_->async_send_request(request, [callback = std::move(callback)](
      rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
    if (callback) callback(future.get()->success);
  });
  return true;
}
}  // namespace offboard_core_pkg
