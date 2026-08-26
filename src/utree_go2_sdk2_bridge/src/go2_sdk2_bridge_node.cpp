#include "utree_go2_sdk2_bridge/go2_sdk2_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <stdexcept>

#include "unitree/robot/channel/channel_factory.hpp"
#include "utree_go2_sdk2_bridge/control_safety.hpp"

namespace utree_go2_sdk2_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kGeometryEpsilon = 1.0e-6;

double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

bool finite(double value)
{
  return std::isfinite(value);
}

}  // namespace

Go2Sdk2BridgeNode::Go2Sdk2BridgeNode() : Node("go2_sdk2_bridge")
{
  const bool configured_enabled = declare_parameter<bool>("enabled", false);
  network_interface_ = declare_parameter("network_interface", "enP8p1s0");
  domain_id_ = declare_parameter("domain_id", 0);
  command_rate_ = declare_parameter("command_rate", 200.0);
  path_timeout_ = declare_parameter("path_timeout", 1.0);
  odom_timeout_ = declare_parameter("odom_timeout", 0.5);
  lookahead_distance_ = declare_parameter("lookahead_distance", 0.35);
  const int truncated_path_sample_count = declare_parameter(
    "truncated_path_sample_count", 8);
  truncated_path_discount_ = declare_parameter("truncated_path_discount", 0.95);
  waypoint_tolerance_ = declare_parameter("waypoint_tolerance", 0.12);
  goal_position_tolerance_ = declare_parameter("goal_position_tolerance", 0.15);
  goal_yaw_tolerance_ = declare_parameter("goal_yaw_tolerance", 0.20);
  heading_alignment_enter_angle_ = declare_parameter(
    "heading_alignment_enter_angle", 0.35);
  heading_alignment_exit_angle_ = declare_parameter(
    "heading_alignment_exit_angle", 0.12);
  explicit_rotation_tolerance_ = declare_parameter(
    "explicit_rotation_tolerance", 0.08);
  translation_speed_ = declare_parameter("translation_speed", 0.20);
  rotation_speed_ = declare_parameter("rotation_speed", 0.30);
  max_vx_ = declare_parameter("max_vx", 0.6);
  max_vy_ = declare_parameter("max_vy", 0.35);
  max_yaw_rate_ = declare_parameter("max_yaw_rate", 0.8);
  world_frame_ = declare_parameter("world_frame", "world");
  body_frame_ = declare_parameter("body_frame", "base_link");
  const std::string path_topic = declare_parameter("path_topic", "/body_path");
  const std::string odom_topic = declare_parameter("odom_topic", "/lio/body_odom");

  if (configured_enabled) {
    throw std::invalid_argument(
            "enabled=true at startup is forbidden; call ~/enable_motion explicitly");
  }
  if (network_interface_.empty()) {
    throw std::invalid_argument("network_interface must name the Go2 NIC");
  }
  if (domain_id_ < 0 || domain_id_ > 232) {
    throw std::invalid_argument("domain_id must be in [0, 232]");
  }
  if (world_frame_.empty() || body_frame_.empty() || path_topic.empty() || odom_topic.empty()) {
    throw std::invalid_argument("world_frame, body_frame, and topics must not be empty");
  }
  if (!finite(command_rate_) || command_rate_ < 20.0 || command_rate_ > 200.0) {
    throw std::invalid_argument("command_rate must be finite and in [20, 200] Hz");
  }
  if (!finite(path_timeout_) || path_timeout_ <= 0.0 || path_timeout_ > 60.0 ||
    !finite(odom_timeout_) || odom_timeout_ <= 0.0 || odom_timeout_ > 60.0)
  {
    throw std::invalid_argument("path_timeout and odom_timeout must be in (0, 60] seconds");
  }
  if (!finite(lookahead_distance_) || lookahead_distance_ <= 0.0 ||
    !finite(waypoint_tolerance_) || waypoint_tolerance_ <= 0.0 ||
    !finite(goal_position_tolerance_) || goal_position_tolerance_ <= 0.0 ||
    !finite(goal_yaw_tolerance_) || goal_yaw_tolerance_ <= 0.0 ||
    goal_yaw_tolerance_ > kPi)
  {
    throw std::invalid_argument("path tolerances and lookahead must be finite and positive");
  }
  if (truncated_path_sample_count < 3 || truncated_path_sample_count > 64 ||
    !finite(truncated_path_discount_) || truncated_path_discount_ <= 0.0 ||
    truncated_path_discount_ > 1.0)
  {
    throw std::invalid_argument(
            "truncated path sampling requires 3..64 samples and discount in (0, 1]");
  }
  truncated_path_sample_count_ = static_cast<std::size_t>(truncated_path_sample_count);
  if (!finite(heading_alignment_enter_angle_) || heading_alignment_enter_angle_ <= 0.0 ||
    heading_alignment_enter_angle_ > kPi ||
    !finite(heading_alignment_exit_angle_) || heading_alignment_exit_angle_ < 0.0 ||
    heading_alignment_exit_angle_ >= heading_alignment_enter_angle_ ||
    !finite(explicit_rotation_tolerance_) || explicit_rotation_tolerance_ <= 0.0 ||
    explicit_rotation_tolerance_ > heading_alignment_enter_angle_)
  {
    throw std::invalid_argument("heading alignment tolerances are invalid");
  }
  if (!finite(translation_speed_) || translation_speed_ <= 0.0 ||
    !finite(rotation_speed_) || rotation_speed_ <= 0.0 ||
    !finite(max_vx_) || max_vx_ <= 0.0 ||
    !finite(max_vy_) || max_vy_ <= 0.0 ||
    !finite(max_yaw_rate_) || max_yaw_rate_ <= 0.0 ||
    translation_speed_ > max_vx_ || rotation_speed_ > max_yaw_rate_)
  {
    throw std::invalid_argument("translation speed and SDK limits are invalid");
  }

  // SDK2 owns its CycloneDDS participant. Initialize it once, then the
  // control timer calls the official synchronous SportClient surface directly.
  unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);
  sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
  sport_client_->SetTimeout(0.5F);
  sport_client_->Init();

  // Do not replay the planner's transient-local cache when the bridge starts.
  // The planner will publish the next live refresh, which is the path the
  // operator actually authorized.
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic, rclcpp::QoS(1).reliable().durability_volatile(),
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
    "Go2 SDK2 path executor on '%s': disabled until ~/enable_motion; "
    "Move is refreshed at %.1f Hz with fixed translation speed %.2f m/s and "
    "official-style arc turns at %.2f rad/s; local path direction uses %zu "
    "discounted samples over %.2f m",
    network_interface_.c_str(), command_rate_, translation_speed_, rotation_speed_,
    truncated_path_sample_count_, lookahead_distance_);
  RCLCPP_INFO(
    get_logger(),
    "Route completion holds Move(0,0,0); only SDK failure, input timeout, /lowcmd, "
    "explicit disable, or shutdown calls StopMove");
}

Go2Sdk2BridgeNode::~Go2Sdk2BridgeNode() noexcept
{
  motion_enabled_ = false;
  if (!sport_client_) {
    return;
  }
  try {
    const int32_t status = sport_client_->StopMove();
    if (status != 0) {
      std::fprintf(stderr, "go2_sdk2_bridge: shutdown StopMove failed with status %d\n", status);
    }
  } catch (...) {
    std::fprintf(stderr, "go2_sdk2_bridge: shutdown StopMove threw\n");
  }
  command_active_ = false;
}

void Go2Sdk2BridgeNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (msg->header.frame_id != world_frame_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Ignoring body path frame '%s'; expected '%s'",
      msg->header.frame_id.c_str(), world_frame_.c_str());
    return;
  }

  if (msg->poses.empty()) {
    // An empty planner path is a normal wait state. Keep authorization and let
    // the next timer tick continue the zero-speed Move stream.
    path_.reset();
    path_refresh_pending_reanchor_ = false;
    path_cursor_index_ = 0U;
    heading_alignment_active_ = false;
    last_path_received_ = std::chrono::steady_clock::now();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Planner path is empty; holding Move(0,0,0) while waiting for a new path");
    return;
  }

  for (std::size_t index = 0; index < msg->poses.size(); ++index) {
    const auto & pose = msg->poses[index];
    if (!pose.header.frame_id.empty() && pose.header.frame_id != world_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring body path pose %zu in frame '%s'; expected '%s'",
        index, pose.header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    if (!isFinitePose(pose.pose)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring body path pose %zu with a non-finite position or quaternion", index);
      return;
    }
  }

  const auto new_generation = pathGoalGeneration(msg->poses);
  const bool same_goal_refresh = path_goal_generation_ && new_generation &&
    *path_goal_generation_ == *new_generation;
  const bool completed_replay = path_waiting_for_new_goal_ &&
    completed_goal_generation_ && new_generation &&
    *completed_goal_generation_ == *new_generation;

  path_ = msg;
  path_goal_generation_ = new_generation;
  last_path_received_ = std::chrono::steady_clock::now();

  if (completed_replay) {
    // Keep the newest copy for diagnostics/freshness, but do not restart a
    // goal that has already reached its endpoint.
    return;
  }

  if (path_waiting_for_new_goal_) {
    path_waiting_for_new_goal_ = false;
    completed_goal_generation_.reset();
    path_cursor_index_ = 0U;
    heading_alignment_active_ = false;
    RCLCPP_INFO(get_logger(), "Accepted a new path after route completion");
  } else if (!same_goal_refresh) {
    path_cursor_index_ = 0U;
    heading_alignment_active_ = false;
  }

  // Every live planner refresh is reconciled against the current odometry on
  // the next control tick. No callback performs an SDK motion RPC.
  path_refresh_pending_reanchor_ = true;
}

void Go2Sdk2BridgeNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (msg->header.frame_id != world_frame_ || msg->child_frame_id != body_frame_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Ignoring odometry frames '%s' -> '%s'; expected '%s' -> '%s'",
      msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
      world_frame_.c_str(), body_frame_.c_str());
    return;
  }
  if (!isFinitePose(msg->pose.pose)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Ignoring odometry with a non-finite position or quaternion");
    return;
  }

  // Use receipt time for the input watchdog. The corrected odometry header can
  // legitimately lag wall time on the Jetson; rejecting it by header age was
  // the source of the previous 0.600 s false stop.
  odom_ = msg;
  last_odom_received_ = std::chrono::steady_clock::now();
}

void Go2Sdk2BridgeNode::enableCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (!request || !response) {
    return;
  }

  if (!request->data) {
    motion_enabled_ = false;
    clearExecutionState();
    const bool stopped = stopRobot("motion disabled by service", true);
    response->success = stopped;
    response->message = stopped ? "Go2 motion disabled" : "StopMove failed; motion disabled";
    return;
  }

  if (lowcmdPublisherPresent()) {
    motion_enabled_ = false;
    clearExecutionState();
    (void)stopRobot("/lowcmd publisher is active while enabling");
    response->success = false;
    response->message = "Cannot enable: a /lowcmd publisher is active";
    return;
  }
  if (!cachedOdomValid() || !odomFresh()) {
    response->success = false;
    response->message = "Cannot enable: waiting for body odometry";
    return;
  }
  if (motion_enabled_) {
    response->success = true;
    response->message = "Go2 motion is already enabled";
    return;
  }
  if (command_active_ && !stopRobot("clearing the previous SDK command before enabling")) {
    response->success = false;
    response->message = "Cannot enable: previous StopMove failed";
    return;
  }

  motion_enabled_ = true;
  response->success = true;
  response->message = path_ ?
    "Go2 motion enabled" : "Go2 motion enabled; waiting for a body path";
}

void Go2Sdk2BridgeNode::controlTick()
{
  try {
    controlTickImpl();
  } catch (const std::exception & exception) {
    disableAfterFault("control loop exception");
    RCLCPP_ERROR(get_logger(), "SDK2 path executor exception: %s", exception.what());
  } catch (...) {
    disableAfterFault("unknown control loop exception");
    RCLCPP_ERROR(get_logger(), "SDK2 path executor failed with an unknown exception");
  }
}

void Go2Sdk2BridgeNode::controlTickImpl()
{
  if (lowcmdPublisherPresent()) {
    if (motion_enabled_ || command_active_) {
      disableAfterFault("/lowcmd publisher appeared");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Stopped SDK2 Move because a /lowcmd publisher is active");
    }
    return;
  }

  if (!motion_enabled_) {
    (void)stopRobot("motion disabled");
    return;
  }
  if (!cachedOdomValid() || !odomFresh()) {
    disableAfterFault("body odometry input timeout");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Stopped SDK2 Move because /lio/body_odom timed out");
    return;
  }

  if (!path_ || path_waiting_for_new_goal_) {
    // This is the official keep-alive pattern: do not transition through
    // StopMove between goals; refresh a zero velocity through Move instead.
    (void)sendMove(0.0, 0.0, 0.0);
    return;
  }
  if (!cachedPathValid() || !pathFresh()) {
    disableAfterFault("body path input timeout");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Stopped SDK2 Move because /body_path timed out or became invalid");
    return;
  }

  if (path_refresh_pending_reanchor_) {
    reanchorPathCursor();
    path_refresh_pending_reanchor_ = false;
  }

  const auto command = makePathCommand();
  if (!command) {
    disableAfterFault("invalid body path command input");
    RCLCPP_ERROR(
      get_logger(), "Stopped SDK2 Move because the active body path cannot be followed");
    return;
  }
  if (command->completed) {
    path_waiting_for_new_goal_ = true;
    completed_goal_generation_ = path_goal_generation_;
    heading_alignment_active_ = false;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Path endpoint reached; holding Move(0,0,0) until a new path arrives");
    (void)sendMove(0.0, 0.0, 0.0);
    return;
  }
  (void)sendMove(command->vx, command->vy, command->yaw_rate);
}

std::optional<Go2Sdk2BridgeNode::PathCommand> Go2Sdk2BridgeNode::makePathCommand()
{
  if (!path_ || path_->poses.empty() || !odom_) {
    return std::nullopt;
  }

  const auto current_yaw = quaternionYaw(odom_->pose.pose.orientation);
  if (!current_yaw) {
    return std::nullopt;
  }
  const double current_x = odom_->pose.pose.position.x;
  const double current_y = odom_->pose.pose.position.y;
  if (!finite(current_x) || !finite(current_y)) {
    return std::nullopt;
  }

  const auto turnCommand = [this](double yaw_error)
    -> std::optional<PathCommand> {
      if (!finite(yaw_error)) {
        return std::nullopt;
      }
      // Unitree's official recurrent velocity example combines forward and
      // yaw motion. Use the configured fixed forward speed during heading
      // alignment as well, instead of relying on a pure-yaw command that the
      // on-robot sport controller has only acknowledged with weak motion.
      const auto bounded = makeBoundedCommand(
        translation_speed_, 0.0, std::copysign(rotation_speed_, yaw_error),
        max_vx_, max_vy_, max_yaw_rate_);
      if (!bounded) {
        return std::nullopt;
      }
      return PathCommand{bounded->vx, bounded->vy, bounded->yaw_rate, false};
    };

  const auto translationCommand = [this, &current_yaw, &turnCommand](double desired_yaw)
    -> std::optional<PathCommand> {
      if (!finite(desired_yaw)) {
        return std::nullopt;
      }
      const double yaw_error = normalizeAngle(desired_yaw - *current_yaw);
      if (!finite(yaw_error)) {
        return std::nullopt;
      }
      const double absolute_error = std::abs(yaw_error);
      if (heading_alignment_active_) {
        if (absolute_error > heading_alignment_exit_angle_) {
          return turnCommand(yaw_error);
        }
        heading_alignment_active_ = false;
      } else if (absolute_error >= heading_alignment_enter_angle_) {
        heading_alignment_active_ = true;
        return turnCommand(yaw_error);
      }

      // Once aligned, continue with the fixed straight-ahead command.
      const auto bounded = makeBoundedCommand(
        translation_speed_, 0.0, 0.0,
        max_vx_, max_vy_, max_yaw_rate_);
      if (!bounded) {
        return std::nullopt;
      }
      return PathCommand{bounded->vx, bounded->vy, bounded->yaw_rate, false};
    };

  const auto & poses = path_->poses;
  if (path_cursor_index_ >= poses.size()) {
    path_cursor_index_ = poses.size() - 1U;
  }

  // Each loop either advances the cursor or returns one command. This handles
  // short connector poses and same-position rotation poses without treating a
  // small geometric offset as a fault.
  for (std::size_t iteration = 0; iteration <= poses.size() + 2U; ++iteration) {
    if (path_cursor_index_ + 1U >= poses.size()) {
      const auto & final_pose = poses.back().pose;
      const double final_dx = final_pose.position.x - current_x;
      const double final_dy = final_pose.position.y - current_y;
      const double final_distance = std::hypot(final_dx, final_dy);
      const auto final_yaw = quaternionYaw(final_pose.orientation);
      if (!final_yaw || !finite(final_distance)) {
        return std::nullopt;
      }
      const double final_yaw_error = normalizeAngle(*final_yaw - *current_yaw);
      if (!finite(final_yaw_error)) {
        return std::nullopt;
      }
      if (final_distance <= goal_position_tolerance_) {
        if (std::abs(final_yaw_error) <= goal_yaw_tolerance_) {
          heading_alignment_active_ = false;
          return PathCommand{0.0, 0.0, 0.0, true};
        }
        heading_alignment_active_ = true;
        return turnCommand(final_yaw_error);
      }

      const double desired_yaw = std::atan2(final_dy, final_dx);
      return translationCommand(desired_yaw);
    }

    const auto & start = poses[path_cursor_index_].pose;
    const auto & end = poses[path_cursor_index_ + 1U].pose;
    const double dx = end.position.x - start.position.x;
    const double dy = end.position.y - start.position.y;
    const double length = std::hypot(dx, dy);
    if (!finite(length)) {
      return std::nullopt;
    }

    const auto end_yaw = quaternionYaw(end.orientation);
    if (!end_yaw) {
      return std::nullopt;
    }

    if (length <= kGeometryEpsilon) {
      const double rotation_error = normalizeAngle(*end_yaw - *current_yaw);
      if (!finite(rotation_error)) {
        return std::nullopt;
      }
      const bool terminal_rotation = path_cursor_index_ + 2U >= poses.size();
      if (terminal_rotation &&
        std::abs(rotation_error) > explicit_rotation_tolerance_)
      {
        heading_alignment_active_ = true;
        return turnCommand(rotation_error);
      }
      // Intermediate same-position yaw states are lattice bookkeeping. The
      // truncated local surrogate below sees the next translating edges and
      // produces one forward arc instead of stopping at every discrete bin.
      ++path_cursor_index_;
      heading_alignment_active_ = false;
      continue;
    }

    const double length_squared = length * length;
    const double projection = (
      (current_x - start.position.x) * dx + (current_y - start.position.y) * dy) /
      length_squared;
    const double endpoint_distance = std::hypot(
      end.position.x - current_x, end.position.y - current_y);
    if (!finite(projection) || !finite(endpoint_distance)) {
      return std::nullopt;
    }
    if (projection >= 1.0 || endpoint_distance <= waypoint_tolerance_) {
      ++path_cursor_index_;
      continue;
    }

    // Approximate the ideal route by a bounded L_T-style local surrogate:
    // sample only the next lookahead prefix, discount those samples, and move
    // toward their weighted direction. This preserves the planner path while
    // avoiding stop-and-turn reactions to every 0.20 m lattice edge.
    const auto surrogate = makeTruncatedPathSurrogate(
      poses, path_cursor_index_, current_x, current_y,
      lookahead_distance_, truncated_path_sample_count_, truncated_path_discount_);
    if (!surrogate) {
      return std::nullopt;
    }
    return translationCommand(surrogate->desired_yaw);
  }

  return std::nullopt;
}

void Go2Sdk2BridgeNode::reanchorPathCursor()
{
  if (!path_ || path_->poses.empty() || !odom_) {
    path_cursor_index_ = 0U;
    return;
  }
  const auto & poses = path_->poses;
  if (poses.size() == 1U) {
    path_cursor_index_ = 0U;
    return;
  }

  const double current_x = odom_->pose.pose.position.x;
  const double current_y = odom_->pose.pose.position.y;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  std::size_t best_index = 0U;
  for (std::size_t index = 0; index + 1U < poses.size(); ++index) {
    const auto & start = poses[index].pose.position;
    const auto & end = poses[index + 1U].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    double projected_x = start.x;
    double projected_y = start.y;
    if (length_squared > kGeometryEpsilon * kGeometryEpsilon) {
      const double projection = std::clamp(
        ((current_x - start.x) * dx + (current_y - start.y) * dy) / length_squared,
        0.0, 1.0);
      projected_x += projection * dx;
      projected_y += projection * dy;
    }
    const double distance_squared =
      (current_x - projected_x) * (current_x - projected_x) +
      (current_y - projected_y) * (current_y - projected_y);
    if (!finite(distance_squared)) {
      continue;
    }
    // At a shared waypoint prefer the later edge so a refreshed path does not
    // replay the short edge that the body has already crossed.
    if (distance_squared < best_distance_squared - 1.0e-10 ||
      (std::abs(distance_squared - best_distance_squared) <= 1.0e-10 &&
      index > best_index))
    {
      best_distance_squared = distance_squared;
      best_index = index;
    }
  }
  path_cursor_index_ = best_index;
}

void Go2Sdk2BridgeNode::clearExecutionState()
{
  path_.reset();
  path_goal_generation_.reset();
  completed_goal_generation_.reset();
  path_cursor_index_ = 0U;
  path_waiting_for_new_goal_ = false;
  path_refresh_pending_reanchor_ = false;
  heading_alignment_active_ = false;
}

void Go2Sdk2BridgeNode::disableAfterFault(const char * reason)
{
  motion_enabled_ = false;
  clearExecutionState();
  (void)stopRobot(reason);
}

bool Go2Sdk2BridgeNode::sendMove(double vx, double vy, double yaw_rate)
{
  if (!sport_client_ || !finite(vx) || !finite(vy) || !finite(yaw_rate)) {
    disableAfterFault("invalid SDK2 Move input");
    return false;
  }

  command_active_ = true;
  int32_t status = -1;
  try {
    // This is intentionally the official synchronous call. The timer invokes
    // it again on the next tick, keeping the robot's short command lease alive.
    status = sport_client_->Move(vx, vy, yaw_rate);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "SportClient::Move threw: %s", exception.what());
    motion_enabled_ = false;
    clearExecutionState();
    (void)stopRobot("SportClient::Move exception");
    return false;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "SportClient::Move threw an unknown exception");
    motion_enabled_ = false;
    clearExecutionState();
    (void)stopRobot("SportClient::Move exception");
    return false;
  }

  if (status != 0) {
    RCLCPP_ERROR(get_logger(), "SportClient::Move failed with status %d", status);
    motion_enabled_ = false;
    clearExecutionState();
    (void)stopRobot("SportClient::Move failure");
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

bool Go2Sdk2BridgeNode::stopRobot(const char * reason, bool force) noexcept
{
  if (force && !command_active_) {
    // An explicit stop is an SDK request even when this process has not yet
    // sent a non-zero command. Keep the retry state if that request fails.
    command_active_ = true;
  }
  if (!command_active_ && !force) {
    return true;
  }
  if (!sport_client_) {
    return false;
  }
  try {
    const int32_t status = sport_client_->StopMove();
    if (status == 0) {
      command_active_ = false;
      RCLCPP_WARN(get_logger(), "Go2 stopped: %s", reason);
      return true;
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "SportClient::StopMove failed with status %d (%s); retrying", status, reason);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "SportClient::StopMove threw (%s): %s; retrying", reason, exception.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "SportClient::StopMove threw (%s); retrying", reason);
  }
  return false;
}

bool Go2Sdk2BridgeNode::cachedPathValid() const
{
  if (!path_ || path_->poses.empty() || path_->header.frame_id != world_frame_) {
    return false;
  }
  return std::all_of(
    path_->poses.begin(), path_->poses.end(),
    [this](const geometry_msgs::msg::PoseStamped & pose) {
      return (pose.header.frame_id.empty() || pose.header.frame_id == world_frame_) &&
             isFinitePose(pose.pose);
    });
}

bool Go2Sdk2BridgeNode::cachedOdomValid() const
{
  return odom_ && odom_->header.frame_id == world_frame_ &&
         odom_->child_frame_id == body_frame_ && isFinitePose(odom_->pose.pose);
}

bool Go2Sdk2BridgeNode::pathFresh() const
{
  if (last_path_received_ == std::chrono::steady_clock::time_point{}) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_path_received_).count();
  return finite(age) && age >= 0.0 && age <= path_timeout_;
}

bool Go2Sdk2BridgeNode::odomFresh() const
{
  if (last_odom_received_ == std::chrono::steady_clock::time_point{}) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_odom_received_).count();
  return finite(age) && age >= 0.0 && age <= odom_timeout_;
}

bool Go2Sdk2BridgeNode::lowcmdPublisherPresent()
{
  return count_publishers("/lowcmd") > 0U;
}

}  // namespace utree_go2_sdk2_bridge
