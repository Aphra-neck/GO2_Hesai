#include "utree_go2_sdk2_bridge/simple_goal_executor_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "unitree/robot/channel/channel_factory.hpp"

namespace utree_go2_sdk2_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kQuaternionEpsilon = 1.0e-9;

bool finite(double value)
{
  return std::isfinite(value);
}

double quaternionYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!finite(norm) || norm <= kQuaternionEpsilon) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y) / norm,
    (norm - 2.0 * (q.y * q.y + q.z * q.z)) / norm);
}

}  // namespace

SimpleGoalExecutorNode::SimpleGoalExecutorNode() : Node("go2_sdk2_direct_bridge")
{
  network_interface_ = declare_parameter("network_interface", "enP8p1s0");
  domain_id_ = declare_parameter("domain_id", 0);
  const bool configured_enabled = declare_parameter("enabled", false);
  command_rate_ = declare_parameter("command_rate", 20.0);
  odom_timeout_ = declare_parameter("odom_timeout", 1.0);
  position_tolerance_ = declare_parameter("position_tolerance", 0.15);
  yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.12);
  align_tolerance_ = declare_parameter("align_tolerance", 0.08);
  linear_gain_ = declare_parameter("linear_gain", 1.0);
  lateral_gain_ = declare_parameter("lateral_gain", 1.0);
  yaw_gain_ = declare_parameter("yaw_gain", 1.5);
  max_vx_ = declare_parameter("max_vx", 0.6);
  max_vy_ = declare_parameter("max_vy", 0.35);
  max_yaw_rate_ = declare_parameter("max_yaw_rate", 0.8);
  world_frame_ = declare_parameter("world_frame", "world");
  body_frame_ = declare_parameter("body_frame", "base_link");
  goal_topic_ = declare_parameter("goal_topic", "/goal_pose");
  odom_topic_ = declare_parameter("odom_topic", "/lio/body_odom");
  command_topic_ = declare_parameter("command_topic", "/sdk2_command");

  if (configured_enabled) {
    throw std::invalid_argument("enabled=true is forbidden; call ~/enable_motion explicitly");
  }
  if (network_interface_.empty() || world_frame_.empty() || body_frame_.empty() ||
    goal_topic_.empty() || odom_topic_.empty() || command_topic_.empty())
  {
    throw std::invalid_argument("network, frame, topic, and interface parameters must be non-empty");
  }
  if (!finite(command_rate_) || command_rate_ <= 0.0 || !finite(odom_timeout_) || odom_timeout_ <= 0.0 ||
    !finite(position_tolerance_) || position_tolerance_ < 0.0 || !finite(yaw_tolerance_) ||
    yaw_tolerance_ < 0.0 || !finite(align_tolerance_) || align_tolerance_ < 0.0 ||
    !finite(linear_gain_) || linear_gain_ <= 0.0 || !finite(lateral_gain_) || lateral_gain_ <= 0.0 ||
    !finite(yaw_gain_) || yaw_gain_ <= 0.0 ||
    !finite(max_vx_) || max_vx_ <= 0.0 || !finite(max_vy_) || max_vy_ <= 0.0 ||
    !finite(max_yaw_rate_) || max_yaw_rate_ <= 0.0)
  {
    throw std::invalid_argument("direct bridge parameters are outside their positive ranges");
  }
  if (domain_id_ < 0 || domain_id_ > 232) {
    throw std::invalid_argument("domain_id must be in [0, 232]");
  }

  unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);
  sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
  sport_client_->SetTimeout(0.5F);
  sport_client_->Init();

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(10),
    std::bind(&SimpleGoalExecutorNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SimpleGoalExecutorNode::odomCallback, this, std::placeholders::_1));
  command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(command_topic_, 10);
  enable_service_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_motion",
    std::bind(
      &SimpleGoalExecutorNode::enableCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  const auto period = std::chrono::duration<double>(1.0 / command_rate_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SimpleGoalExecutorNode::controlTick, this));

  RCLCPP_WARN(
    get_logger(),
    "Direct Go2 SDK2 bridge is disabled; it uses /goal_pose and /lio/body_odom, "
    "ignores /body_path, and does not perform obstacle avoidance");
}

SimpleGoalExecutorNode::~SimpleGoalExecutorNode() noexcept
{
  if (!sport_client_ || !command_active_) {
    return;
  }
  try {
    (void)sport_client_->StopMove();
  } catch (...) {
  }
}

void SimpleGoalExecutorNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const double yaw = quaternionYaw(msg->pose.orientation);
  if (!finite(msg->pose.position.x) || !finite(msg->pose.position.y) || !finite(yaw)) {
    RCLCPP_ERROR(get_logger(), "Ignoring goal with a non-finite position or quaternion");
    return;
  }
  if (!msg->header.frame_id.empty() && msg->header.frame_id != world_frame_) {
    RCLCPP_ERROR(
      get_logger(), "Ignoring goal in frame '%s'; direct bridge requires '%s'",
      msg->header.frame_id.c_str(), world_frame_.c_str());
    return;
  }
  goal_ = *msg;
  route_.clear();
  route_index_ = 0;
  phase_ = Phase::kAlignSegment;
  if (odom_ && finiteOdom()) {
    rebuildRoute();
  }
  RCLCPP_INFO(
    get_logger(), "Accepted simple goal (%.3f, %.3f); route will be rebuilt from current odometry",
    msg->pose.position.x, msg->pose.position.y);
}

void SimpleGoalExecutorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  odom_ = *msg;
  last_odom_received_ = std::chrono::steady_clock::now();
}

void SimpleGoalExecutorNode::enableCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (!request || !response) {
    return;
  }
  if (!request->data) {
    armed_ = false;
    clearGoal("motion disabled");
    response->success = stopRobot("motion disabled");
    response->message = response->success ? "Direct Go2 bridge disabled" : "StopMove not confirmed";
    return;
  }
  if (lowcmdPublisherPresent()) {
    response->success = false;
    response->message = "Cannot enable: a /lowcmd publisher is active";
    return;
  }
  if (!odom_ || !odomFresh() || !finiteOdom()) {
    response->success = false;
    response->message = "Cannot enable: waiting for body odometry";
    return;
  }
  armed_ = true;
  response->success = true;
  response->message = goal_ ? "Direct Go2 bridge armed" : "Direct Go2 bridge armed; waiting for /goal_pose";
}

void SimpleGoalExecutorNode::controlTick()
{
  try {
    controlTickImpl();
  } catch (const std::exception & exception) {
    armed_ = false;
    clearGoal("control exception");
    stopRobot("control exception");
    RCLCPP_ERROR(get_logger(), "Simple navigation control exception: %s", exception.what());
  } catch (...) {
    armed_ = false;
    clearGoal("unknown control exception");
    stopRobot("unknown control exception");
    RCLCPP_ERROR(get_logger(), "Simple navigation control failed");
  }
}

void SimpleGoalExecutorNode::controlTickImpl()
{
  if (lowcmdPublisherPresent()) {
    armed_ = false;
    clearGoal("/lowcmd publisher appeared");
    stopRobot("/lowcmd publisher appeared");
    return;
  }
  if (!armed_) {
    stopRobot("motion disabled");
    return;
  }
  if (!odom_ || !odomFresh() || !finiteOdom()) {
    armed_ = false;
    clearGoal("body odometry stopped");
    stopRobot("body odometry stopped");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Direct bridge stopped: body odometry stopped");
    return;
  }
  if (!goal_) {
    stopRobot("waiting for goal");
    return;
  }
  if (route_.empty() && route_index_ == 0) {
    rebuildRoute();
  }
  if (!finiteGoal()) {
    armed_ = false;
    clearGoal("invalid goal");
    stopRobot("invalid goal");
    return;
  }

  advanceReachedSegments();
  if (route_index_ >= route_.size()) {
    const auto yaw = currentYaw();
    const auto target_yaw = goalYaw();
    if (!yaw || !target_yaw) {
      armed_ = false;
      clearGoal("invalid heading");
      stopRobot("invalid heading");
      return;
    }
    const double error = normalizeAngle(*target_yaw - *yaw);
    if (std::abs(error) <= yaw_tolerance_) {
      stopRobot("simple goal reached");
      clearGoal("simple goal reached");
      RCLCPP_INFO(get_logger(), "Direct-bridge goal reached; authorization remains armed");
      return;
    }
    (void)sendMove(0.0, 0.0, clamp(yaw_gain_ * error, max_yaw_rate_));
    return;
  }

  if (phase_ == Phase::kAlignSegment) {
    const auto desired = desiredSegmentYaw();
    const auto yaw = currentYaw();
    if (!desired || !yaw) {
      armed_ = false;
      clearGoal("invalid segment heading");
      stopRobot("invalid segment heading");
      return;
    }
    const double error = normalizeAngle(*desired - *yaw);
    if (std::abs(error) <= align_tolerance_) {
      phase_ = Phase::kTranslateSegment;
      // Keep the SDK2 motion stream alive across an internal phase change.
      // StopMove() can leave the Go2 waiting for a fresh locomotion wake-up;
      // it is reserved for goal completion, disable, timeout, and faults.
      RCLCPP_INFO(
        get_logger(), "Direct bridge segment heading aligned; continuing with translation");
      return;
    }
    (void)sendMove(0.0, 0.0, clamp(yaw_gain_ * error, max_yaw_rate_));
    return;
  }

  const auto & target = route_[route_index_];
  const double start_x = has_segment_start_ ? segment_start_.x : odom_->pose.pose.position.x;
  const double start_y = has_segment_start_ ? segment_start_.y : odom_->pose.pose.position.y;
  const double segment_dx = target.x - start_x;
  const double segment_dy = target.y - start_y;
  const double segment_length = std::hypot(segment_dx, segment_dy);
  if (!finite(segment_length) || segment_length <= position_tolerance_) {
    advanceReachedSegments();
    return;
  }
  const double direction_x = segment_dx / segment_length;
  const double direction_y = segment_dy / segment_length;
  const double current_progress =
    (odom_->pose.pose.position.x - start_x) * direction_x +
    (odom_->pose.pose.position.y - start_y) * direction_y;
  const double remaining = std::clamp(segment_length - current_progress, 0.0, segment_length);
  const double lateral_error =
    (odom_->pose.pose.position.x - start_x) * (-direction_y) +
    (odom_->pose.pose.position.y - start_y) * direction_x;
  const double world_dx = direction_x * remaining;
  const double world_dy = direction_y * remaining;
  const double correction_x = direction_y * lateral_error;
  const double correction_y = -direction_x * lateral_error;
  const auto yaw = currentYaw();
  if (!yaw) {
    armed_ = false;
    clearGoal("invalid body heading");
    stopRobot("invalid body heading");
    return;
  }
  const double cos_yaw = std::cos(*yaw);
  const double sin_yaw = std::sin(*yaw);
  const double body_dx = cos_yaw * (world_dx + correction_x) + sin_yaw * (world_dy + correction_y);
  const double body_dy = -sin_yaw * (world_dx + correction_x) + cos_yaw * (world_dy + correction_y);
  const auto desired = desiredSegmentYaw();
  const double yaw_error = desired ? normalizeAngle(*desired - *yaw) : 0.0;
  (void)sendMove(
    clamp(linear_gain_ * body_dx, max_vx_),
    clamp(lateral_gain_ * body_dy, max_vy_),
    clamp(yaw_gain_ * yaw_error, max_yaw_rate_));
}

bool SimpleGoalExecutorNode::lowcmdPublisherPresent()
{
  return count_publishers("/lowcmd") > 0U;
}

bool SimpleGoalExecutorNode::odomFresh() const
{
  if (last_odom_received_ == std::chrono::steady_clock::time_point{}) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_odom_received_).count();
  return finite(age) && age >= 0.0 && age <= odom_timeout_;
}

bool SimpleGoalExecutorNode::finiteOdom() const
{
  if (!odom_) {
    return false;
  }
  const auto & p = odom_->pose.pose.position;
  return finite(p.x) && finite(p.y) && finite(p.z) && currentYaw().has_value();
}

bool SimpleGoalExecutorNode::finiteGoal() const
{
  return goal_ && finite(goal_->pose.position.x) && finite(goal_->pose.position.y) && goalYaw().has_value();
}

bool SimpleGoalExecutorNode::stopRobot(const char * reason) noexcept
{
  if (!command_active_) {
    return true;
  }
  try {
    const int32_t status = sport_client_->StopMove();
    if (status == 0) {
      command_active_ = false;
      RCLCPP_INFO(get_logger(), "Direct bridge stopped: %s", reason);
      return true;
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Direct-bridge StopMove failed with status %d (%s)", status, reason);
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Direct-bridge StopMove threw (%s)", reason);
  }
  return false;
}

bool SimpleGoalExecutorNode::sendMove(double vx, double vy, double yaw_rate)
{
  command_active_ = true;
  int32_t status = -1;
  try {
    status = sport_client_->Move(vx, vy, yaw_rate);
  } catch (...) {
    armed_ = false;
    clearGoal("SportClient::Move threw");
    stopRobot("SportClient::Move threw");
    return false;
  }
  if (status != 0) {
    armed_ = false;
    clearGoal("SportClient::Move failed");
    stopRobot("SportClient::Move failed");
    RCLCPP_ERROR(get_logger(), "Direct-bridge SportClient::Move failed with status %d", status);
    return false;
  }
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = body_frame_;
  command.twist.linear.x = vx;
  command.twist.linear.y = vy;
  command.twist.angular.z = yaw_rate;
  command_pub_->publish(command);
  return true;
}

void SimpleGoalExecutorNode::clearGoal(const char * reason)
{
  if (goal_) {
    RCLCPP_INFO(get_logger(), "Direct-bridge goal cleared: %s", reason);
  }
  goal_.reset();
  route_.clear();
  route_index_ = 0;
  has_segment_start_ = false;
  phase_ = Phase::kAlignSegment;
}

void SimpleGoalExecutorNode::rebuildRoute()
{
  if (!goal_ || !odom_ || !finiteOdom() || !finiteGoal()) {
    return;
  }
  route_ = makeRightAngleRoute(
    odom_->pose.pose.position.x, odom_->pose.pose.position.y, *currentYaw(),
    goal_->pose.position.x, goal_->pose.position.y, position_tolerance_);
  route_index_ = 0;
  segment_start_ = {odom_->pose.pose.position.x, odom_->pose.pose.position.y};
  has_segment_start_ = true;
  phase_ = Phase::kAlignSegment;
  RCLCPP_INFO(get_logger(), "Direct bridge route contains %zu translation targets", route_.size());
}

void SimpleGoalExecutorNode::advanceReachedSegments()
{
  while (route_index_ < route_.size()) {
    const auto & target = route_[route_index_];
    const double distance = std::hypot(
      target.x - odom_->pose.pose.position.x,
      target.y - odom_->pose.pose.position.y);
    if (!finite(distance) || distance > position_tolerance_) {
      return;
    }
    ++route_index_;
    segment_start_ = target;
    has_segment_start_ = true;
    phase_ = Phase::kAlignSegment;
    RCLCPP_INFO(
      get_logger(), "Direct bridge right-angle waypoint reached; continuing with next segment");
  }
}

std::optional<double> SimpleGoalExecutorNode::currentYaw() const
{
  if (!odom_) {
    return std::nullopt;
  }
  const double value = quaternionYaw(odom_->pose.pose.orientation);
  return finite(value) ? std::optional<double>(value) : std::nullopt;
}

std::optional<double> SimpleGoalExecutorNode::goalYaw() const
{
  if (!goal_) {
    return std::nullopt;
  }
  const double value = quaternionYaw(goal_->pose.orientation);
  return finite(value) ? std::optional<double>(value) : std::nullopt;
}

std::optional<double> SimpleGoalExecutorNode::desiredSegmentYaw() const
{
  if (!odom_ || route_index_ >= route_.size()) {
    return std::nullopt;
  }
  const auto & target = route_[route_index_];
  const double start_x = has_segment_start_ ? segment_start_.x : odom_->pose.pose.position.x;
  const double start_y = has_segment_start_ ? segment_start_.y : odom_->pose.pose.position.y;
  const double dx = target.x - start_x;
  const double dy = target.y - start_y;
  if (std::hypot(dx, dy) <= position_tolerance_) {
    return std::nullopt;
  }
  return std::atan2(dy, dx);
}

double SimpleGoalExecutorNode::normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double SimpleGoalExecutorNode::clamp(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

}  // namespace utree_go2_sdk2_bridge
