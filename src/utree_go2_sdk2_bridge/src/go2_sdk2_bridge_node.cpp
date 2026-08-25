#include "utree_go2_sdk2_bridge/go2_sdk2_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/go2/sport/sport_client.hpp"

namespace utree_go2_sdk2_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

class SportClientCommandBackend final : public SdkCommandBackend
{
public:
  explicit SportClientCommandBackend(
    std::unique_ptr<unitree::robot::go2::SportClient> sport_client)
  : sport_client_(std::move(sport_client))
  {
  }

  std::int32_t move(const SdkVelocityCommand & command) override
  {
    return sport_client_->Move(command.vx, command.vy, command.yaw_rate);
  }

  std::int32_t stop() override
  {
    return sport_client_->StopMove();
  }

private:
  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
};
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
  sport_state_timeout_ = declare_parameter("sport_state_timeout", 1.0);
  timestamp_future_tolerance_ = declare_parameter("timestamp_future_tolerance", 0.2);
  lookahead_distance_ = declare_parameter("lookahead_distance", 0.6);
  goal_position_tolerance_ = declare_parameter("goal_position_tolerance", 0.15);
  goal_yaw_tolerance_ = declare_parameter("goal_yaw_tolerance", 0.20);
  heading_alignment_enter_angle_ =
    declare_parameter("heading_alignment_enter_angle", 0.7853981633974483);
  heading_alignment_exit_angle_ =
    declare_parameter("heading_alignment_exit_angle", 0.2617993877991494);
  explicit_rotation_tolerance_ =
    declare_parameter("explicit_rotation_tolerance", 0.05);
  linear_gain_ = declare_parameter("linear_gain", 1.0);
  yaw_gain_ = declare_parameter("yaw_gain", 1.5);
  rcl_interfaces::msg::ParameterDescriptor velocity_limit_descriptor;
  velocity_limit_descriptor.description =
    "Read-only operating limit loaded from YAML or an explicit launch override";
  velocity_limit_descriptor.read_only = true;
  max_vx_ = declare_parameter<double>("max_vx", velocity_limit_descriptor);
  max_vy_ = declare_parameter<double>("max_vy", velocity_limit_descriptor);
  max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", velocity_limit_descriptor);
  rcl_interfaces::msg::ParameterDescriptor response_descriptor;
  response_descriptor.description =
    "Read-only low-speed execution and physical-response safety setting";
  response_descriptor.read_only = true;
  minimum_translation_speed_ =
    declare_parameter<double>("minimum_translation_speed", 0.20, response_descriptor);
  motion_response_timeout_ =
    declare_parameter<double>("motion_response_timeout", 2.0, response_descriptor);
  motion_response_min_translation_ = declare_parameter<double>(
    "motion_response_min_translation", 0.04, response_descriptor);
  motion_response_min_yaw_ =
    declare_parameter<double>("motion_response_min_yaw", 0.05, response_descriptor);
  world_frame_ = declare_parameter("world_frame", "world");
  body_frame_ = declare_parameter("body_frame", "base_link");
  const std::string path_topic = declare_parameter("path_topic", "/body_path");
  const std::string odom_topic = declare_parameter("odom_topic", "/lio/body_odom");

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
  if (world_frame_.empty() || body_frame_.empty() || path_topic.empty() || odom_topic.empty()) {
    throw std::invalid_argument(
            "world_frame, body_frame, path_topic, and odom_topic must not be empty");
  }
  const ControlParameters parameters{
    command_rate_, path_timeout_, odom_timeout_, sport_state_timeout_, timestamp_future_tolerance_,
    lookahead_distance_, goal_position_tolerance_, goal_yaw_tolerance_,
    heading_alignment_enter_angle_, heading_alignment_exit_angle_,
    explicit_rotation_tolerance_,
    linear_gain_, yaw_gain_, minimum_translation_speed_, motion_response_timeout_,
    motion_response_min_translation_, motion_response_min_yaw_,
    max_vx_, max_vy_, max_yaw_rate_};
  const std::string parameter_error = validateControlParameters(parameters);
  if (!parameter_error.empty()) {
    throw std::invalid_argument(parameter_error);
  }
  // SDK2 owns its DDS participant. Initialize it once before constructing SportClient.
  unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);
  auto sport_client = std::make_unique<unitree::robot::go2::SportClient>();
  sport_client->SetTimeout(0.5F);
  sport_client->Init();
  command_worker_ = std::make_unique<SdkCommandWorker>(
    std::make_shared<SportClientCommandBackend>(std::move(sport_client)));

  sport_state_sub_ = std::make_shared<
    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
    "rt/sportmodestate");
  sport_state_sub_->InitChannel(
    std::bind(&Go2Sdk2BridgeNode::sportStateCallback, this, std::placeholders::_1), 1);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    // Execute only paths published after this bridge subscription is matched.
    // The planner remains transient-local for RViz, but replaying its cached path
    // here can disarm a freshly armed bridge before the operator sends a new goal.
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
    "Go2 SDK2 bridge on interface '%s'; motion is disabled and can only be enabled via "
    "~/enable_motion",
    network_interface_.c_str());
}

Go2Sdk2BridgeNode::~Go2Sdk2BridgeNode() noexcept
{
  if (!command_worker_) {
    return;
  }

  try {
    motion_authorization_.disarm();
    command_worker_->shutdown();
    const auto status = command_worker_->status();
    if (status.stop_state == SdkStopState::kConfirmed) {
      command_active_ = false;
      RCLCPP_WARN(get_logger(), "Go2 stopped during SDK2 bridge shutdown");
    } else {
      RCLCPP_FATAL(get_logger(), "Unable to confirm StopMove during SDK2 bridge shutdown");
    }
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
      // A path callback never submits Move directly. The next fully gated
      // control tick holds zero while preserving the operator authorization.
      path_progress_tracker_.reset();
      if (!waitForNewPath("empty path")) {
        RCLCPP_ERROR(
          get_logger(), "Body path cleared, but StopMove remains unconfirmed while disarmed");
        return;
      }
      if (motion_authorization_.armed()) {
        RCLCPP_INFO(
          get_logger(), "Body path cleared; waiting for a new path while remaining armed");
      } else {
        RCLCPP_WARN(get_logger(), "Body path cleared while motion is disarmed");
      }
      return;
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
    const auto goal_generation = pathGoalGeneration(msg->poses);
    if (!goal_generation) {
      failSafe("invalid path goal generation");
      RCLCPP_ERROR(
        get_logger(),
        "Rejected body path because pose stamps do not contain one valid goal generation");
      return;
    }
    switch (completed_goal_latch_.evaluate(*goal_generation)) {
      case GoalGenerationDecision::kAccept:
        break;
      case GoalGenerationDecision::kCompletedReplay:
        path_progress_tracker_.reset();
        if (!waitForNewPath("completed goal generation replay")) {
          RCLCPP_ERROR(
            get_logger(), "Completed goal replay was rejected, but StopMove remains unconfirmed");
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Ignored body path for completed goal generation %lld",
          static_cast<long long>(*goal_generation));
        return;
      case GoalGenerationDecision::kSuperseded:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Ignored superseded body path generation %lld without interrupting the active path",
          static_cast<long long>(*goal_generation));
        return;
      case GoalGenerationDecision::kInvalid:
        failSafe("invalid path goal generation decision");
        RCLCPP_ERROR(
          get_logger(), "Rejected invalid body path goal generation %lld",
          static_cast<long long>(*goal_generation));
        return;
    }
    path_ = msg;
    path_goal_generation_ = *goal_generation;
    ++accepted_path_sequence_;
    path_progress_tracker_.reset();
    motion_authorization_.pathAvailable();
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
    if (msg->child_frame_id != body_frame_) {
      failSafe("odometry body frame mismatch");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected odometry child frame '%s'; expected '%s'",
        msg->child_frame_id.c_str(), body_frame_.c_str());
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
    if (odom_) {
      const rclcpp::Time previous_time(odom_->header.stamp, current_time.get_clock_type());
      if (message_time <= previous_time) {
        failSafe("nonmonotonic odometry timestamp");
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Rejected nonmonotonic odometry timestamp: delta %.6f s",
          (message_time - previous_time).seconds());
        return;
      }
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

void Go2Sdk2BridgeNode::sportStateCallback(const void * message)
{
  if (!message) {
    return;
  }
  const auto * state =
    static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
  const SportStateSample sample{
    state->error_code(), state->mode(), state->gait_type()};
  const std::lock_guard<std::mutex> lock(sport_state_mutex_);
  sport_state_ = sample;
  unsafe_sport_state_latch_.observe(sample);
  sport_state_received_at_ = std::chrono::steady_clock::now();
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
    const bool sdk_completions_healthy = processSdkCompletions();
    if (!request->data) {
      motion_authorization_.disarm();
      path_.reset();
      path_goal_generation_.reset();
      path_progress_tracker_.reset();
      motion_response_watchdog_.reset();
      completed_goal_latch_.clear();
      heading_alignment_active_ = false;
      const bool stopped = stopRobot("motion disabled by service");
      const auto worker_status = command_worker_->status();
      const bool stop_queued =
        worker_status.stop_state == SdkStopState::kPending && pending_stop_sequence_.has_value();
      response->success = stopped;
      response->message = stopped ? "Go2 motion disabled" :
        (stop_queued ? "Motion authorization disabled; StopMove confirmation is pending" :
        "Disable requested, but StopMove could not be queued");
      return;
    }
    if (!sdk_completions_healthy) {
      response->success = false;
      response->message =
        "Cannot arm: an SDK2 command failure was processed; verify state and call again";
      return;
    }
    if (lowcmdPublisherPresent()) {
      failSafe("a /lowcmd publisher is active");
      response->success = false;
      response->message = "Cannot enable: a /lowcmd publisher is active";
      return;
    }
    if (!motion_authorization_.armed() && command_active_) {
      response->success = false;
      response->message = "Cannot enable: waiting for StopMove confirmation";
      return;
    }
    const rclcpp::Time current_time = now();
    if (!cachedOdomValid() || !odomFresh(current_time)) {
      failSafe("invalid or stale odometry while enabling motion");
      response->success = false;
      response->message = "Cannot arm: odometry is invalid, missing, or stale";
      return;
    }
    if (path_ && !cachedPathValid()) {
      failSafe("invalid path while enabling motion");
      response->success = false;
      response->message = "Cannot arm: cached body path is invalid";
      return;
    }
    if (path_ && !pathFresh(current_time)) {
      if (motion_authorization_.armed()) {
        failSafe("path timeout while enabling motion");
        response->success = false;
        response->message =
          "Cannot remain armed: cached body path timed out; re-arm after a fresh path";
        return;
      }
      if (!waitForNewPath("discarding stale path before arming")) {
        response->success = false;
        response->message = "Cannot arm: StopMove is not confirmed";
        return;
      }
    }
    const auto sport_state = freshSportState();
    if (!sport_state) {
      failSafe("missing or stale Unitree sport state while enabling motion");
      response->success = false;
      response->message = "Cannot arm: rt/sportmodestate is missing or stale";
      return;
    }
    if (!isExecutableSportState(sport_state->state_code)) {
      failSafe("Unitree sport state is not executable while enabling motion");
      response->success = false;
      response->message =
        "Cannot arm: Unitree motion state " + std::to_string(sport_state->state_code) +
        " (" + sportStateName(sport_state->state_code) + ") is not executable";
      return;
    }
    if (motion_authorization_.armed()) {
      response->success = true;
      response->message = motion_authorization_.executionAuthorized() ?
        "Go2 motion is already armed" : "Go2 motion is already armed and waiting for a path";
      return;
    }
    if (!command_worker_->resetFaultAfterConfirmedStop()) {
      response->success = false;
      response->message = "Cannot arm: SDK2 StopMove is not confirmed";
      return;
    }
    motion_authorization_.arm(path_ != nullptr);
    motion_response_watchdog_.reset();
    response->success = true;
    response->message = path_ ?
      "Go2 motion armed" : "Go2 motion armed; waiting for a fresh body path";
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
  if (!processSdkCompletions()) {
    return;
  }
  if (lowcmdPublisherPresent()) {
    failSafe("a /lowcmd publisher appeared");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "A /lowcmd publisher is active; SportClient motion is disabled");
    return;
  }
  if (!motion_authorization_.armed()) {
    stopRobot("retrying unconfirmed stop while motion is disabled");
    return;
  }
  const auto sport_state = freshSportState();
  if (!sport_state) {
    failSafe("Unitree sport state timeout");
    RCLCPP_ERROR(get_logger(), "Stopped because rt/sportmodestate is missing or stale");
    return;
  }
  if (!isExecutableSportState(sport_state->state_code)) {
    failSafe("Unitree sport state became non-executable");
    RCLCPP_ERROR(
      get_logger(), "Stopped in Unitree motion state %u (%s), mode=%u gait_type=%u",
      sport_state->state_code, sportStateName(sport_state->state_code),
      static_cast<unsigned>(sport_state->mode),
      static_cast<unsigned>(sport_state->gait_type));
    return;
  }
  const rclcpp::Time current_time = now();
  if (!cachedOdomValid()) {
    failSafe("odometry became invalid");
    RCLCPP_ERROR(get_logger(), "Cached odometry failed safety validation");
    return;
  }
  if (!odomFresh(current_time)) {
    failSafe("odometry timeout");
    return;
  }
  if (!path_) {
    if (!holdZeroMoveWhileWaiting("waiting for a path")) {
      motion_authorization_.disarm();
      odom_.reset();
    }
    return;
  }
  if (!cachedPathValid()) {
    failSafe("path became invalid");
    RCLCPP_ERROR(get_logger(), "Cached path failed safety validation");
    return;
  }
  if (!pathFresh(current_time)) {
    failSafe("path timeout");
    return;
  }
  if (!motion_authorization_.executionAuthorized()) {
    motion_authorization_.pathAvailable();
  }
  if (!motion_authorization_.executionAuthorized()) {
    failSafe("path execution was not authorized");
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
  PathTrackingDiagnostics tracking_diagnostics;
  const auto tracking_target = path_progress_tracker_.update(
    path_->poses, current.position.x, current.position.y, *current_yaw,
    lookahead_distance_, explicit_rotation_tolerance_, &tracking_diagnostics);
  const auto completion_ready = goalCompletionReady(
    goal_distance, goal_yaw_error, goal_position_tolerance_, goal_yaw_tolerance_,
    tracking_target);
  if (!completion_ready) {
    const rclcpp::Time path_time(path_->header.stamp, current_time.get_clock_type());
    const double path_age = messageAgeSeconds(path_time, current_time);
    const std::size_t path_pose_count = path_->poses.size();
    const std::int64_t goal_generation = path_goal_generation_.value_or(-1);
    failSafe("path progress could not be confirmed");
    RCLCPP_ERROR(
      get_logger(),
      "Stopped because bounded monotonic path progress could not be confirmed: "
      "failure=%s target=%s path_sequence=%llu goal_generation=%lld path_age=%.6f "
      "pose_count=%zu tracker_initialized=%s tracker_pose=%zu tracker_fraction=%.9f "
      "current_progress=%.9f cross_track=%.9f projection_distance=%.9f "
      "waypoint_distance=%.9f final_distance=%.9f previous_progress=%.9f "
      "maximum_progress=%.9f projected_progress=%.9f block_end=%zu block_length=%.9f "
      "current=(%.9f,%.9f,%.9f) segment=(%.9f,%.9f)->(%.9f,%.9f) "
      "path_start=(%.9f,%.9f) path_final=(%.9f,%.9f)",
      pathTrackingFailureName(tracking_diagnostics.failure),
      tracking_target ? "present" : "absent",
      static_cast<unsigned long long>(accepted_path_sequence_),
      static_cast<long long>(goal_generation), path_age, path_pose_count,
      tracking_diagnostics.tracker_initialized ? "true" : "false",
      tracking_diagnostics.pose_index, tracking_diagnostics.segment_fraction,
      tracking_diagnostics.current_progress, tracking_diagnostics.cross_track,
      tracking_diagnostics.projection_distance, tracking_diagnostics.waypoint_distance,
      tracking_diagnostics.final_distance, tracking_diagnostics.previous_progress,
      tracking_diagnostics.maximum_progress, tracking_diagnostics.projected_progress,
      tracking_diagnostics.block_end, tracking_diagnostics.block_length,
      tracking_diagnostics.current_x, tracking_diagnostics.current_y,
      tracking_diagnostics.current_yaw, tracking_diagnostics.segment_start_x,
      tracking_diagnostics.segment_start_y, tracking_diagnostics.segment_end_x,
      tracking_diagnostics.segment_end_y, tracking_diagnostics.path_start_x,
      tracking_diagnostics.path_start_y, tracking_diagnostics.path_final_x,
      tracking_diagnostics.path_final_y);
    return;
  }
  if (*completion_ready) {
    if (!path_goal_generation_) {
      failSafe("missing path goal generation at completion");
      return;
    }
    completed_goal_latch_.markCompleted(*path_goal_generation_);
    (void)waitForNewPath("goal reached");
    return;
  }
  const double world_dx = tracking_target->target_x - current.position.x;
  const double world_dy = tracking_target->target_y - current.position.y;
  const double cos_yaw = std::cos(*current_yaw);
  const double sin_yaw = std::sin(*current_yaw);
  const double body_dx = cos_yaw * world_dx + sin_yaw * world_dy;
  const double body_dy = -sin_yaw * world_dx + cos_yaw * world_dy;
  const auto local_heading_yaw = quaternionYaw(
    path_->poses[tracking_target->heading_pose].pose.orientation);
  if (!local_heading_yaw) {
    failSafe("invalid target quaternion during command calculation");
    return;
  }
  const auto target_yaw_error = selectAlignmentYawError(
    normalizeAngle(*local_heading_yaw - *current_yaw), goal_yaw_error,
    goal_distance, goal_position_tolerance_,
    tracking_target->explicit_rotation_waypoint,
    tracking_target->pending_explicit_rotation);
  if (!target_yaw_error) {
    failSafe("invalid target heading selection");
    return;
  }

  const auto gate_active = updateHeadingAlignmentGate(
    heading_alignment_active_, *target_yaw_error,
    heading_alignment_enter_angle_, heading_alignment_exit_angle_);
  if (!gate_active) {
    failSafe("invalid heading alignment calculation");
    return;
  }
  const auto rotate_in_place = requireRotateInPlace(
    *gate_active, tracking_target->explicit_rotation_waypoint, *target_yaw_error,
    explicit_rotation_tolerance_, goal_distance, goal_position_tolerance_,
    tracking_target->pending_explicit_rotation);
  if (!rotate_in_place) {
    failSafe("invalid rotate-in-place decision");
    return;
  }
  heading_alignment_active_ = *rotate_in_place;

  const auto raw_vx = filterLongitudinalCommand(
    *rotate_in_place ? 0.0 : linear_gain_ * body_dx,
    tracking_target->translation_direction);
  if (!raw_vx) {
    failSafe("unplanned reverse command");
    RCLCPP_ERROR(
      get_logger(), "Rejected negative vx because the active path segment is not reverse");
    return;
  }
  const double raw_vy = linear_gain_ * body_dy;
  const double raw_yaw_rate = yaw_gain_ * (*target_yaw_error);
  const auto bounded_command = makeHeadingAwareCommand(
    *raw_vx, raw_vy, raw_yaw_rate, heading_alignment_active_,
    max_vx_, max_vy_, max_yaw_rate_);
  if (!bounded_command) {
    failSafe("non-finite or invalid velocity command");
    RCLCPP_ERROR(get_logger(), "Rejected unsafe velocity command before SportClient::Move");
    return;
  }
  const auto command = applyMinimumPlanarSpeed(
    *bounded_command, minimum_translation_speed_, max_vx_, max_vy_);
  if (!command) {
    failSafe("invalid minimum-speed command");
    RCLCPP_ERROR(get_logger(), "Rejected invalid minimum-speed command before SportClient::Move");
    return;
  }

  const double steady_time = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto response_healthy = motion_response_watchdog_.observe(
    *command, current.position.x, current.position.y, *current_yaw, steady_time,
    motion_response_timeout_, motion_response_min_translation_, motion_response_min_yaw_);
  if (!response_healthy) {
    failSafe("invalid motion response watchdog state");
    RCLCPP_ERROR(get_logger(), "Motion response watchdog rejected invalid state");
    return;
  }
  if (!*response_healthy) {
    failSafe("motion response timeout");
    RCLCPP_ERROR(
      get_logger(),
      "Stopped after %.2f s without %.3f m/%.3f rad of odometry response",
      motion_response_timeout_, motion_response_min_translation_, motion_response_min_yaw_);
    return;
  }

  if (!sendMove(command->vx, command->vy, command->yaw_rate)) {
    failSafe("SDK2 Move could not be queued");
    return;
  }
}

bool Go2Sdk2BridgeNode::processSdkCompletions()
{
  if (!command_worker_) {
    return false;
  }

  bool healthy = true;
  SdkCommandCompletion completion;
  while (command_worker_->tryPopCompletion(completion)) {
    if (completion.outcome == SdkCommandOutcome::kSuperseded ||
      completion.outcome == SdkCommandOutcome::kDiscarded ||
      completion.outcome == SdkCommandOutcome::kPreemptedByStop)
    {
      continue;
    }

    if (completion.kind == SdkCommandKind::kStop) {
      if (pending_stop_sequence_ && *pending_stop_sequence_ == completion.sequence) {
        pending_stop_sequence_.reset();
      }
      if (completion.outcome == SdkCommandOutcome::kSucceeded) {
        command_active_ = false;
        const std::string reason = pending_stop_reason_.empty() ?
          "SDK2 stop request" : pending_stop_reason_;
        pending_stop_reason_.clear();
        RCLCPP_WARN(get_logger(), "Go2 stopped: %s", reason.c_str());
      } else {
        healthy = false;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "SportClient::StopMove worker failed with status %d; will retry",
          completion.sdk_status);
      }
      continue;
    }

    if (completion.outcome == SdkCommandOutcome::kSucceeded) {
      geometry_msgs::msg::TwistStamped command_message;
      command_message.header.stamp = now();
      command_message.header.frame_id = body_frame_;
      command_message.twist.linear.x = completion.command.vx;
      command_message.twist.linear.y = completion.command.vy;
      command_message.twist.angular.z = completion.command.yaw_rate;
      command_pub_->publish(command_message);
      continue;
    }

    healthy = false;
    RCLCPP_ERROR(
      get_logger(), "SportClient::Move worker failed with status %d%s",
      completion.sdk_status,
      completion.outcome == SdkCommandOutcome::kSdkException ? " after an exception" : "");
    failSafe("SDK2 Move worker reported an error");
  }
  return healthy;
}

void Go2Sdk2BridgeNode::failSafe(const char * reason)
{
  motion_authorization_.disarm();
  path_.reset();
  path_goal_generation_.reset();
  odom_.reset();
  path_progress_tracker_.reset();
  motion_response_watchdog_.reset();
  completed_goal_latch_.clear();
  heading_alignment_active_ = false;
  stopRobot(reason);
}

bool Go2Sdk2BridgeNode::waitForNewPath(const char * reason)
{
  if (command_worker_) {
    // Prevent a coalesced nonzero command from outliving the path that
    // produced it. This changes only the worker mailbox and performs no RPC.
    (void)command_worker_->discardPendingMove();
  }
  path_.reset();
  path_goal_generation_.reset();
  path_progress_tracker_.reset();
  motion_response_watchdog_.reset();
  motion_authorization_.waitForPath();
  heading_alignment_active_ = false;

  if (motion_authorization_.armed()) {
    // This helper is also called from subscription callbacks. Never enqueue a
    // Move here: the next control tick must pass every safety gate first.
    return true;
  }
  return stopRobot(reason);
}

bool Go2Sdk2BridgeNode::holdZeroMoveWhileWaiting(const char * reason)
{
  motion_response_watchdog_.reset();
  if (!motion_authorization_.armed()) {
    return stopRobot(reason);
  }

  // Keep the locomotion stream active without a StopMove transition. This is
  // an A/B test for the observed wake symptom; hardware feedback must confirm
  // whether it changes physical execution.
  if (sendMove(0.0, 0.0, 0.0)) {
    return true;
  }

  stopRobot("zero-speed wait command failed");
  return false;
}

bool Go2Sdk2BridgeNode::sendMove(double vx, double vy, double yaw_rate)
{
  if (!command_worker_) {
    return false;
  }
  try {
    const auto sequence = command_worker_->submitMove(
      SdkVelocityCommand{
        static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(yaw_rate)});
    if (!sequence) {
      RCLCPP_ERROR(get_logger(), "SDK2 worker rejected Move while stopping or faulted");
      return false;
    }
    // The RPC may reach the robot before its completion is observed. Treat the
    // command as active as soon as it enters the serialized SDK mailbox.
    command_active_ = true;
    return true;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Could not queue SDK2 Move: %s", exception.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Could not queue SDK2 Move after an unknown exception");
    return false;
  }
}

bool Go2Sdk2BridgeNode::stopRobot(const char * reason) noexcept
{
  if (!command_active_) {return true;}
  try {
    if (pending_stop_sequence_) {
      return false;
    }
    const auto sequence = command_worker_->submitStop();
    if (!sequence) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "SDK2 worker rejected StopMove (%s); will retry", reason);
      return false;
    }
    pending_stop_sequence_ = *sequence;
    if (pending_stop_reason_.empty()) {
      pending_stop_reason_ = reason;
    }
    return false;
  } catch (const std::exception & exception) {
    try {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Could not queue SDK2 StopMove (%s): %s; will retry", reason, exception.what());
    } catch (...) {
      std::fprintf(stderr, "go2_sdk2_bridge: could not queue StopMove: %s\n", exception.what());
    }
    return false;
  } catch (...) {
    try {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Could not queue SDK2 StopMove after an unknown exception (%s); will retry", reason);
    } catch (...) {
      std::fprintf(stderr, "go2_sdk2_bridge: could not queue StopMove\n");
    }
    return false;
  }
}

bool Go2Sdk2BridgeNode::cachedPathValid() const
{
  if (!path_ || path_->poses.empty() || path_->header.frame_id != world_frame_)
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

bool Go2Sdk2BridgeNode::cachedOdomValid() const
{
  return odom_ && odom_->header.frame_id == world_frame_ &&
         odom_->child_frame_id == body_frame_ && isFinitePose(odom_->pose.pose);
}

bool Go2Sdk2BridgeNode::pathFresh(const rclcpp::Time & current_time) const
{
  const auto clock_type = current_time.get_clock_type();
  return path_ && messageStampFresh(
           rclcpp::Time(path_->header.stamp, clock_type), current_time, path_timeout_);
}

bool Go2Sdk2BridgeNode::odomFresh(const rclcpp::Time & current_time) const
{
  const auto clock_type = current_time.get_clock_type();
  return odom_ && messageStampFresh(
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

std::optional<SportStateSample> Go2Sdk2BridgeNode::freshSportState()
{
  SportStateSample sample{};
  std::chrono::steady_clock::time_point received_at;
  {
    const std::lock_guard<std::mutex> lock(sport_state_mutex_);
    if (motion_authorization_.armed()) {
      if (const auto unsafe_sample = unsafe_sport_state_latch_.take()) {
        return unsafe_sample;
      }
    } else {
      (void)unsafe_sport_state_latch_.take();
    }
    if (!sport_state_) {
      return std::nullopt;
    }
    sample = *sport_state_;
    received_at = sport_state_received_at_;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - received_at).count();
  if (!std::isfinite(age) || age < 0.0 || age > sport_state_timeout_) {
    return std::nullopt;
  }
  return sample;
}

bool Go2Sdk2BridgeNode::lowcmdPublisherPresent()
{
  return count_publishers("/lowcmd") > 0U;
}

}  // namespace utree_go2_sdk2_bridge
