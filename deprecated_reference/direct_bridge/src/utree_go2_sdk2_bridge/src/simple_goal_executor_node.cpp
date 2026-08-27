#include "utree_go2_sdk2_bridge/simple_goal_executor_node.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "unitree/robot/channel/channel_factory.hpp"

namespace utree_go2_sdk2_bridge
{
namespace
{
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
  waypoint_cross_track_tolerance_ = declare_parameter("waypoint_cross_track_tolerance", 0.30);
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
    !finite(waypoint_cross_track_tolerance_) || waypoint_cross_track_tolerance_ < 0.0 ||
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

  navigation_.setConfig({
      position_tolerance_, yaw_tolerance_, align_tolerance_,
      waypoint_cross_track_tolerance_, linear_gain_, lateral_gain_, yaw_gain_,
      max_vx_, max_vy_, max_yaw_rate_});

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
  navigation_.setGoal({msg->pose.position.x, msg->pose.position.y, yaw});
  const auto current_yaw = currentYaw();
  if (odom_ && current_yaw) {
    navigation_.prepareRoute({
        odom_->pose.pose.position.x, odom_->pose.pose.position.y, *current_yaw});
  }
  RCLCPP_INFO(
    get_logger(), "Accepted simple goal (%.3f, %.3f); route will be rebuilt from current odometry",
    msg->pose.position.x, msg->pose.position.y);
  RCLCPP_INFO(
    get_logger(), "Direct bridge route contains %zu translation targets", navigation_.route().size());
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
  response->message = navigation_.hasGoal() ?
    "Direct Go2 bridge armed" : "Direct Go2 bridge armed; waiting for /goal_pose";
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
  if (!navigation_.hasGoal()) {
    // Hold a zero-speed Move while armed between goals. StopMove() can leave
    // this Go2 firmware waiting for a fresh locomotion wake-up.
    (void)sendMove(0.0, 0.0, 0.0);
    return;
  }
  const auto yaw = currentYaw();
  if (!yaw) {
    armed_ = false;
    clearGoal("invalid body heading");
    stopRobot("invalid body heading");
    return;
  }
  const auto result = navigation_.update({
      odom_->pose.pose.position.x, odom_->pose.pose.position.y, *yaw});
  for (std::size_t i = 0; i < result.waypoints_reached; ++i) {
    RCLCPP_INFO(
      get_logger(), "Direct bridge right-angle waypoint reached; continuing with next segment");
  }
  if (result.segment_aligned) {
    RCLCPP_INFO(
      get_logger(), "Direct bridge segment heading aligned; continuing with translation");
  }
  if (!result.valid) {
    armed_ = false;
    clearGoal("invalid navigation command");
    stopRobot("invalid navigation command");
    return;
  }
  if (!sendMove(result.vx, result.vy, result.yaw_rate)) {
    return;
  }
  if (result.goal_reached) {
    RCLCPP_INFO(get_logger(), "Direct-bridge goal reached; authorization remains armed");
  }
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
  if (navigation_.hasGoal()) {
    RCLCPP_INFO(get_logger(), "Direct-bridge goal cleared: %s", reason);
  }
  navigation_.clearGoal();
}

std::optional<double> SimpleGoalExecutorNode::currentYaw() const
{
  if (!odom_) {
    return std::nullopt;
  }
  const double value = quaternionYaw(odom_->pose.pose.orientation);
  return finite(value) ? std::optional<double>(value) : std::nullopt;
}

}  // namespace utree_go2_sdk2_bridge
