#include "utree_go2_sdk2_bridge/go2_sdk2_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <stdexcept>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "unitree/robot/channel/channel_factory.hpp"

namespace utree_go2_sdk2_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}
}  // namespace

Go2Sdk2BridgeNode::Go2Sdk2BridgeNode() : Node("go2_sdk2_bridge")
{
  network_interface_ = declare_parameter("network_interface", "enP8p1s0");
  domain_id_ = declare_parameter("domain_id", 0);
  rcl_interfaces::msg::ParameterDescriptor enabled_descriptor;
  enabled_descriptor.description =
    "Deprecated startup setting; must remain false. Use ~/enable_motion at runtime.";
  enabled_descriptor.read_only = true;
  const bool configured_enabled =
    declare_parameter<bool>("enabled", false, enabled_descriptor);
  command_rate_ = declare_parameter("command_rate", 20.0);
  path_timeout_ = declare_parameter("path_timeout", 1.0);
  odom_timeout_ = declare_parameter("odom_timeout", 0.5);
  timestamp_future_tolerance_ = declare_parameter("timestamp_future_tolerance", 0.2);
  lookahead_distance_ = declare_parameter("lookahead_distance", 0.6);
  goal_position_tolerance_ = declare_parameter("goal_position_tolerance", 0.15);
  goal_yaw_tolerance_ = declare_parameter("goal_yaw_tolerance", 0.20);
  linear_gain_ = declare_parameter("linear_gain", 1.0);
  yaw_gain_ = declare_parameter("yaw_gain", 1.5);
  max_vx_ = declare_parameter("max_vx", 0.6);
  max_vy_ = declare_parameter("max_vy", 0.35);
  max_yaw_rate_ = declare_parameter("max_yaw_rate", 0.8);
  world_frame_ = declare_parameter("world_frame", "world");
  const std::string path_topic = declare_parameter("path_topic", "/body_path");
  const std::string odom_topic = declare_parameter("odom_topic", "/lio/odom");

  if (configured_enabled) {
    throw std::invalid_argument(
            "enabled=true at startup is forbidden; start disabled and call "
            "~/enable_motion only after the operator verifies the robot is safe");
  }
  if (network_interface_.empty()) {
    throw std::invalid_argument("network_interface must name the NIC connected to the Go2");
  }
  if (domain_id_ < 0 || domain_id_ > 232) {
    throw std::invalid_argument("domain_id must be in [0, 232]");
  }
  if (world_frame_.empty() || path_topic.empty() || odom_topic.empty()) {
    throw std::invalid_argument("world_frame, path_topic, and odom_topic must not be empty");
  }
  const ControlParameters parameters{
    command_rate_, path_timeout_, odom_timeout_, timestamp_future_tolerance_,
    lookahead_distance_, goal_position_tolerance_, goal_yaw_tolerance_, linear_gain_,
    yaw_gain_, max_vx_, max_vy_, max_yaw_rate_};
  const std::string parameter_error = validateControlParameters(parameters);
  if (!parameter_error.empty()) {
    throw std::invalid_argument(parameter_error);
  }
  enabled_ = false;

  // SDK2 owns its DDS participant. Initialize it once before constructing SportClient.
  unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);
  sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
  sport_client_->SetTimeout(0.5F);
  sport_client_->Init();

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Go2Sdk2BridgeNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&Go2Sdk2BridgeNode::odomCallback, this, std::placeholders::_1));
  command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("sdk2_command", 10);
  enable_service_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_motion",
    std::bind(
      &Go2Sdk2BridgeNode::enableCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  const auto period = std::chrono::duration<double>(1.0 / command_rate_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&Go2Sdk2BridgeNode::controlTick, this));

  RCLCPP_WARN(
    get_logger(),
    "Go2 SDK2 bridge on interface '%s'; motion is disabled and can only be enabled via "
    "~/enable_motion",
    network_interface_.c_str());
}

Go2Sdk2BridgeNode::~Go2Sdk2BridgeNode() noexcept
{
  if (!sport_client_ || !command_active_) {
    return;
  }

  constexpr int kStopAttempts = 3;
  try {
    for (int attempt = 1; attempt <= kStopAttempts; ++attempt) {
      try {
        const int32_t status = sport_client_->StopMove();
        if (status == 0) {
          command_active_ = false;
          RCLCPP_WARN(get_logger(), "Go2 stopped during SDK2 bridge shutdown");
          return;
        }
        RCLCPP_ERROR(
          get_logger(), "Shutdown StopMove attempt %d/%d failed with status %d",
          attempt, kStopAttempts, status);
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          get_logger(), "Shutdown StopMove attempt %d/%d threw: %s",
          attempt, kStopAttempts, exception.what());
      } catch (...) {
        RCLCPP_ERROR(
          get_logger(), "Shutdown StopMove attempt %d/%d threw an unknown exception",
          attempt, kStopAttempts);
      }
    }
    RCLCPP_FATAL(
      get_logger(), "Unable to confirm StopMove after %d shutdown attempts",
      kStopAttempts);
  } catch (const std::exception & exception) {
    std::fprintf(
      stderr, "go2_sdk2_bridge: shutdown stop failed while logging: %s\n", exception.what());
  } catch (...) {
    std::fprintf(stderr, "go2_sdk2_bridge: shutdown stop failed with an unknown exception\n");
  }
}

void Go2Sdk2BridgeNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  try {
    if (!msg) {
      failSafe("null path message");
      RCLCPP_ERROR(get_logger(), "Rejected null path message");
      return;
    }
    if (msg->header.frame_id != world_frame_) {
      failSafe("path frame mismatch");
      RCLCPP_ERROR(
        get_logger(), "Rejected path frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    if (msg->poses.empty()) {
      failSafe("empty path");
      RCLCPP_ERROR(get_logger(), "Rejected empty body path");
      return;
    }
    for (std::size_t index = 0; index < msg->poses.size(); ++index) {
      const auto & stamped_pose = msg->poses[index];
      if (!stamped_pose.header.frame_id.empty() &&
        stamped_pose.header.frame_id != world_frame_)
      {
        failSafe("path pose frame mismatch");
        RCLCPP_ERROR(
          get_logger(), "Rejected path pose %zu frame '%s'; expected empty or '%s'",
          index, stamped_pose.header.frame_id.c_str(), world_frame_.c_str());
        return;
      }
      if (!isFinitePose(stamped_pose.pose)) {
        failSafe("invalid path pose");
        RCLCPP_ERROR(
          get_logger(), "Rejected path pose %zu: position or quaternion is invalid", index);
        return;
      }
    }
    const rclcpp::Time current_time = now();
    const rclcpp::Time message_time(msg->header.stamp, current_time.get_clock_type());
    if (!messageStampFresh(message_time, current_time, path_timeout_)) {
      const double age = messageAgeSeconds(message_time, current_time);
      failSafe("stale or future path timestamp");
      RCLCPP_ERROR(
        get_logger(),
        "Rejected path with age %.3f s; accepted range is [-%.3f, %.3f] s",
        age, timestamp_future_tolerance_, path_timeout_);
      return;
    }
    path_ = msg;
  } catch (const std::exception & exception) {
    failSafe("exception while validating path");
    RCLCPP_ERROR(get_logger(), "Rejected path after exception: %s", exception.what());
  } catch (...) {
    failSafe("unknown exception while validating path");
    RCLCPP_ERROR(get_logger(), "Rejected path after an unknown exception");
  }
}

void Go2Sdk2BridgeNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  try {
    if (!msg) {
      failSafe("null odometry message");
      RCLCPP_ERROR(get_logger(), "Rejected null odometry message");
      return;
    }
    if (msg->header.frame_id != world_frame_) {
      failSafe("odometry frame mismatch");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected odometry frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    if (!isFinitePose(msg->pose.pose)) {
      failSafe("invalid odometry pose");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected odometry: position or quaternion is invalid");
      return;
    }
    const rclcpp::Time current_time = now();
    const rclcpp::Time message_time(msg->header.stamp, current_time.get_clock_type());
    if (!messageStampFresh(message_time, current_time, odom_timeout_)) {
      const double age = messageAgeSeconds(message_time, current_time);
      failSafe("stale or future odometry timestamp");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected odometry with age %.3f s; accepted range is [-%.3f, %.3f] s",
        age, timestamp_future_tolerance_, odom_timeout_);
      return;
    }
    odom_ = msg;
  } catch (const std::exception & exception) {
    failSafe("exception while validating odometry");
    RCLCPP_ERROR(get_logger(), "Rejected odometry after exception: %s", exception.what());
  } catch (...) {
    failSafe("unknown exception while validating odometry");
    RCLCPP_ERROR(get_logger(), "Rejected odometry after an unknown exception");
  }
}

void Go2Sdk2BridgeNode::enableCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  try {
    if (!request || !response) {
      failSafe("invalid enable service request");
      return;
    }
    if (!request->data) {
      enabled_ = false;
      stopRobot("motion disabled by service");
      response->success = !command_active_;
      response->message = command_active_ ?
        "Disable requested, but StopMove is not yet confirmed" : "Go2 motion disabled";
      return;
    }
    if (lowcmdPublisherPresent()) {
      failSafe("a /lowcmd publisher is active");
      response->success = false;
      response->message = "Cannot enable: a /lowcmd publisher is active";
      return;
    }
    if (command_active_) {
      response->success = false;
      response->message = "Cannot enable: waiting for StopMove confirmation";
      return;
    }
    if (!cachedInputsValid() || !inputsFresh(now())) {
      failSafe("invalid or stale inputs while enabling motion");
      response->success = false;
      response->message = "Cannot enable: body_path or odometry is invalid/missing/stale";
      return;
    }
    enabled_ = true;
    response->success = true;
    response->message = "Go2 motion enabled";
  } catch (const std::exception & exception) {
    failSafe("exception while changing motion state");
    if (response) {
      response->success = false;
      response->message = std::string("Motion remains disabled: ") + exception.what();
    }
    RCLCPP_ERROR(get_logger(), "Motion enable service failed: %s", exception.what());
  } catch (...) {
    failSafe("unknown exception while changing motion state");
    if (response) {
      response->success = false;
      response->message = "Motion remains disabled after an unknown exception";
    }
    RCLCPP_ERROR(get_logger(), "Motion enable service failed with an unknown exception");
  }
}

void Go2Sdk2BridgeNode::controlTick()
{
  try {
    controlTickImpl();
  } catch (const std::exception & exception) {
    failSafe("exception in control loop");
    RCLCPP_ERROR(get_logger(), "Control loop exception: %s", exception.what());
  } catch (...) {
    failSafe("unknown exception in control loop");
    RCLCPP_ERROR(get_logger(), "Control loop failed with an unknown exception");
  }
}

void Go2Sdk2BridgeNode::controlTickImpl()
{
  if (lowcmdPublisherPresent()) {
    failSafe("a /lowcmd publisher appeared");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "A /lowcmd publisher is active; SportClient motion is disabled");
    return;
  }
  if (!enabled_) {
    stopRobot("retrying unconfirmed stop while motion is disabled");
    return;
  }
  const rclcpp::Time current_time = now();
  if (!cachedInputsValid()) {
    failSafe("path or odometry became invalid");
    RCLCPP_ERROR(get_logger(), "Cached path or odometry failed safety validation");
    return;
  }
  if (!inputsFresh(current_time)) {
    failSafe("path or odometry timeout");
    return;
  }

  const auto & current = odom_->pose.pose;
  const auto & goal = path_->poses.back().pose;
  const auto current_yaw = quaternionYaw(current.orientation);
  const auto goal_yaw = quaternionYaw(goal.orientation);
  if (!current_yaw || !goal_yaw) {
    failSafe("invalid quaternion during goal calculation");
    return;
  }
  const double goal_dx = goal.position.x - current.position.x;
  const double goal_dy = goal.position.y - current.position.y;
  const double goal_distance = std::hypot(goal_dx, goal_dy);
  const double goal_yaw_error = normalizeAngle(*goal_yaw - *current_yaw);
  if (!std::isfinite(goal_dx) || !std::isfinite(goal_dy) ||
    !std::isfinite(goal_distance) || !std::isfinite(goal_yaw_error))
  {
    failSafe("non-finite goal calculation");
    return;
  }
  if (goal_distance <= goal_position_tolerance_ &&
    std::abs(goal_yaw_error) <= goal_yaw_tolerance_)
  {
    enabled_ = false;
    stopRobot("goal reached");
    return;
  }

  const auto & target = path_->poses[selectLookaheadPose()].pose;
  const double world_dx = target.position.x - current.position.x;
  const double world_dy = target.position.y - current.position.y;
  const double cos_yaw = std::cos(*current_yaw);
  const double sin_yaw = std::sin(*current_yaw);
  const double body_dx = cos_yaw * world_dx + sin_yaw * world_dy;
  const double body_dy = -sin_yaw * world_dx + cos_yaw * world_dy;
  const auto target_yaw = quaternionYaw(target.orientation);
  if (!target_yaw) {
    failSafe("invalid target quaternion during command calculation");
    return;
  }
  const double target_yaw_error = normalizeAngle(*target_yaw - *current_yaw);

  const double raw_vx = linear_gain_ * body_dx;
  const double raw_vy = linear_gain_ * body_dy;
  const double raw_yaw_rate = yaw_gain_ * target_yaw_error;
  const auto command = makeBoundedCommand(
    raw_vx, raw_vy, raw_yaw_rate, max_vx_, max_vy_, max_yaw_rate_);
  if (!command) {
    failSafe("non-finite or invalid velocity command");
    RCLCPP_ERROR(get_logger(), "Rejected unsafe velocity command before SportClient::Move");
    return;
  }

  // Move may have reached the robot even when its RPC reports an error. Mark the
  // command active first so every failure path issues StopMove conservatively.
  command_active_ = true;
  int32_t status = -1;
  try {
    status = sport_client_->Move(command->vx, command->vy, command->yaw_rate);
  } catch (const std::exception & exception) {
    failSafe("SDK2 Move threw an exception");
    RCLCPP_ERROR(get_logger(), "SportClient::Move threw: %s", exception.what());
    return;
  } catch (...) {
    failSafe("SDK2 Move threw an unknown exception");
    RCLCPP_ERROR(get_logger(), "SportClient::Move threw an unknown exception");
    return;
  }
  if (status != 0) {
    failSafe("SDK2 Move returned an error");
    RCLCPP_ERROR(get_logger(), "SportClient::Move failed with status %d", status);
    return;
  }

  geometry_msgs::msg::TwistStamped command_message;
  command_message.header.stamp = current_time;
  command_message.header.frame_id = "base_link";
  command_message.twist.linear.x = command->vx;
  command_message.twist.linear.y = command->vy;
  command_message.twist.angular.z = command->yaw_rate;
  command_pub_->publish(command_message);
}

void Go2Sdk2BridgeNode::failSafe(const char * reason)
{
  enabled_ = false;
  path_.reset();
  odom_.reset();
  stopRobot(reason);
}

void Go2Sdk2BridgeNode::stopRobot(const char * reason) noexcept
{
  if (!command_active_) {return;}
  try {
    const int32_t status = sport_client_->StopMove();
    if (status != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "SportClient::StopMove failed with status %d (%s); will retry",
        status, reason);
      return;
    }
    command_active_ = false;
    RCLCPP_WARN(get_logger(), "Go2 stopped: %s", reason);
  } catch (const std::exception & exception) {
    try {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "SportClient::StopMove threw (%s): %s; will retry", reason, exception.what());
    } catch (...) {
      std::fprintf(stderr, "go2_sdk2_bridge: StopMove threw: %s\n", exception.what());
    }
  } catch (...) {
    try {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "SportClient::StopMove threw an unknown exception (%s); will retry", reason);
    } catch (...) {
      std::fprintf(stderr, "go2_sdk2_bridge: StopMove threw an unknown exception\n");
    }
  }
}

bool Go2Sdk2BridgeNode::cachedInputsValid() const
{
  if (!path_ || !odom_ || path_->poses.empty() ||
    path_->header.frame_id != world_frame_ || odom_->header.frame_id != world_frame_ ||
    !isFinitePose(odom_->pose.pose))
  {
    return false;
  }
  return std::all_of(
    path_->poses.begin(), path_->poses.end(),
    [this](const geometry_msgs::msg::PoseStamped & pose) {
      return (pose.header.frame_id.empty() || pose.header.frame_id == world_frame_) &&
             isFinitePose(pose.pose);
    });
}

bool Go2Sdk2BridgeNode::inputsFresh(const rclcpp::Time & current_time) const
{
  const auto clock_type = current_time.get_clock_type();
  return path_ && odom_ &&
         messageStampFresh(
           rclcpp::Time(path_->header.stamp, clock_type), current_time, path_timeout_) &&
         messageStampFresh(
           rclcpp::Time(odom_->header.stamp, clock_type), current_time, odom_timeout_);
}

double Go2Sdk2BridgeNode::messageAgeSeconds(
  const rclcpp::Time & message_time,
  const rclcpp::Time & current_time) const
{
  return (current_time - message_time).seconds();
}

bool Go2Sdk2BridgeNode::messageStampFresh(
  const rclcpp::Time & message_time,
  const rclcpp::Time & current_time,
  double timeout) const
{
  const double age = messageAgeSeconds(message_time, current_time);
  return std::isfinite(age) && age >= -timestamp_future_tolerance_ && age <= timeout;
}

bool Go2Sdk2BridgeNode::lowcmdPublisherPresent()
{
  return count_publishers("/lowcmd") > 0U;
}

std::size_t Go2Sdk2BridgeNode::selectLookaheadPose() const
{
  const auto & position = odom_->pose.pose.position;
  std::size_t closest = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path_->poses.size(); ++i) {
    const auto & point = path_->poses[i].pose.position;
    const double distance = std::hypot(point.x - position.x, point.y - position.y);
    if (distance < closest_distance) {
      closest = i;
      closest_distance = distance;
    }
  }
  double accumulated = 0.0;
  for (std::size_t i = closest + 1; i < path_->poses.size(); ++i) {
    const auto & previous = path_->poses[i - 1].pose.position;
    const auto & current = path_->poses[i].pose.position;
    accumulated += std::hypot(current.x - previous.x, current.y - previous.y);
    if (accumulated >= lookahead_distance_) {return i;}
  }
  return path_->poses.size() - 1;
}

}  // namespace utree_go2_sdk2_bridge
