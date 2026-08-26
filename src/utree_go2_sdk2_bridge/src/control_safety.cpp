#include "utree_go2_sdk2_bridge/control_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace utree_go2_sdk2_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

bool inClosedRange(double value, double minimum, double maximum)
{
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

bool inPositiveRange(double value, double maximum)
{
  return std::isfinite(value) && value > 0.0 && value <= maximum;
}
}  // namespace

bool isExecutableSportState(std::uint32_t state_code)
{
  return state_code == 100U || state_code == 1013U;
}

const char * sportStateName(std::uint32_t state_code)
{
  switch (state_code) {
    case 100U: return "agile";
    case 1001U: return "damping";
    case 1002U: return "standing lock";
    case 1004U:
    case 2006U: return "crouch";
    case 1006U: return "special action";
    case 1007U: return "sit";
    case 1008U: return "front jump";
    case 1009U: return "front pounce";
    case 1013U: return "balance standing";
    case 1015U: return "regular walking";
    case 1016U: return "regular running";
    case 1017U: return "regular endurance";
    case 1091U: return "pose";
    case 2007U: return "avoidance";
    case 2008U: return "bound run";
    case 2009U: return "jump run";
    case 2010U: return "classic walk";
    case 2011U: return "handstand";
    case 2012U: return "front flip";
    case 2013U: return "back flip";
    case 2014U: return "left flip";
    case 2016U: return "cross step";
    case 2017U: return "upright";
    case 2019U: return "towing";
    default: return "unknown";
  }
}

void UnsafeSportStateLatch::observe(const SportStateSample & sample)
{
  if (!isExecutableSportState(sample.state_code) && !pending_sample_) {
    pending_sample_ = sample;
  }
}

std::optional<SportStateSample> UnsafeSportStateLatch::take()
{
  const auto sample = pending_sample_;
  pending_sample_.reset();
  return sample;
}

MotionAuthorizationState MotionAuthorization::state() const
{
  return state_;
}

bool MotionAuthorization::armed() const
{
  return state_ != MotionAuthorizationState::kDisarmed;
}

bool MotionAuthorization::executionAuthorized() const
{
  return state_ == MotionAuthorizationState::kArmedExecuting;
}

void MotionAuthorization::arm(bool path_available)
{
  state_ = path_available ? MotionAuthorizationState::kArmedExecuting :
    MotionAuthorizationState::kArmedWaitingForPath;
}

void MotionAuthorization::pathAvailable()
{
  if (armed()) {
    state_ = MotionAuthorizationState::kArmedExecuting;
  }
}

void MotionAuthorization::waitForPath()
{
  if (armed()) {
    state_ = MotionAuthorizationState::kArmedWaitingForPath;
  }
}

void MotionAuthorization::disarm()
{
  state_ = MotionAuthorizationState::kDisarmed;
}

std::string validateControlParameters(const ControlParameters & parameters)
{
  // These are hard rejection limits, not operating recommendations. Runtime
  // defaults live in the package YAML and remain below the SDK capability envelope.
  if (!inClosedRange(parameters.command_rate, 1.0, 200.0)) {
    return "command_rate must be finite and in [1, 200] Hz";
  }
  if (!inPositiveRange(parameters.path_timeout, 60.0)) {
    return "path_timeout must be finite and in (0, 60] seconds";
  }
  if (!inPositiveRange(parameters.odom_timeout, 60.0)) {
    return "odom_timeout must be finite and in (0, 60] seconds";
  }
  if (!inPositiveRange(parameters.sport_state_timeout, 60.0)) {
    return "sport_state_timeout must be finite and in (0, 60] seconds";
  }
  if (!inClosedRange(parameters.timestamp_future_tolerance, 0.0, 5.0)) {
    return "timestamp_future_tolerance must be finite and in [0, 5] seconds";
  }
  if (!inPositiveRange(parameters.lookahead_distance, 10.0)) {
    return "lookahead_distance must be finite and in (0, 10] metres";
  }
  if (!inPositiveRange(parameters.goal_position_tolerance, 2.0)) {
    return "goal_position_tolerance must be finite and in (0, 2] metres";
  }
  if (!inPositiveRange(parameters.goal_yaw_tolerance, kPi)) {
    return "goal_yaw_tolerance must be finite and in (0, pi] radians";
  }
  if (!inPositiveRange(parameters.heading_alignment_enter_angle, kPi)) {
    return "heading_alignment_enter_angle must be finite and in (0, pi] radians";
  }
  if (!inClosedRange(
      parameters.heading_alignment_exit_angle, 0.0,
      parameters.heading_alignment_enter_angle) ||
    parameters.heading_alignment_exit_angle >= parameters.heading_alignment_enter_angle)
  {
    return "heading_alignment_exit_angle must be finite and in [0, enter_angle) radians";
  }
  if (!inPositiveRange(
      parameters.explicit_rotation_tolerance,
      parameters.heading_alignment_exit_angle) ||
    parameters.explicit_rotation_tolerance >= parameters.heading_alignment_exit_angle)
  {
    return "explicit_rotation_tolerance must be finite and in (0, exit_angle) radians";
  }
  if (!inClosedRange(parameters.linear_gain, 0.0, 20.0)) {
    return "linear_gain must be finite and in [0, 20]";
  }
  if (!inClosedRange(parameters.yaw_gain, 0.0, 20.0)) {
    return "yaw_gain must be finite and in [0, 20]";
  }
  if (!inPositiveRange(parameters.max_vx, kSdkMaxSymmetricVx)) {
    return "max_vx must be finite and in (0, 2.5] m/s because it bounds both directions";
  }
  if (!inPositiveRange(parameters.max_vy, kSdkMaxAbsVy)) {
    return "max_vy must be finite and in (0, 1] m/s";
  }
  if (!inPositiveRange(parameters.max_yaw_rate, kSdkMaxAbsYawRate)) {
    return "max_yaw_rate must be finite and in (0, 4] rad/s";
  }
  const double minimum_axis_limit = std::min(parameters.max_vx, parameters.max_vy);
  if (!inPositiveRange(parameters.minimum_translation_speed, minimum_axis_limit)) {
    return "minimum_translation_speed must be finite, positive, and no greater than both "
           "max_vx and max_vy";
  }
  if (!inPositiveRange(parameters.motion_response_timeout, 10.0)) {
    return "motion_response_timeout must be finite and in (0, 10] seconds";
  }
  if (!inPositiveRange(
      parameters.motion_response_min_translation,
      parameters.minimum_translation_speed * parameters.motion_response_timeout))
  {
    return "motion_response_min_translation must be finite, positive, and reachable at the "
           "minimum translation speed within the response timeout";
  }
  const double maximum_response_yaw = std::min(
    kPi, parameters.max_yaw_rate * parameters.motion_response_timeout);
  if (!inPositiveRange(parameters.motion_response_min_yaw, maximum_response_yaw))
  {
    return "motion_response_min_yaw must be finite, positive, no greater than pi, and "
           "reachable at max_yaw_rate within the response timeout";
  }
  return {};
}

bool isValidQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
  {
    return false;
  }

  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) &&
         std::abs(norm_squared - 1.0) <= kQuaternionNormTolerance;
}

bool isFinitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         isValidQuaternion(pose.orientation);
}

std::optional<double> quaternionYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!isValidQuaternion(quaternion)) {
    return std::nullopt;
  }

  const double yaw = std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
  if (!std::isfinite(yaw)) {
    return std::nullopt;
  }
  return yaw;
}

std::optional<VelocityCommand> makeBoundedCommand(
  double raw_vx,
  double raw_vy,
  double raw_yaw_rate,
  double max_vx,
  double max_vy,
  double max_yaw_rate)
{
  if (!std::isfinite(raw_vx) || !std::isfinite(raw_vy) ||
    !std::isfinite(raw_yaw_rate) ||
    !inPositiveRange(max_vx, std::numeric_limits<float>::max()) ||
    !inPositiveRange(max_vy, std::numeric_limits<float>::max()) ||
    !inPositiveRange(max_yaw_rate, std::numeric_limits<float>::max()))
  {
    return std::nullopt;
  }

  const VelocityCommand command{
    static_cast<float>(std::clamp(raw_vx, -max_vx, max_vx)),
    static_cast<float>(std::clamp(raw_vy, -max_vy, max_vy)),
    static_cast<float>(std::clamp(raw_yaw_rate, -max_yaw_rate, max_yaw_rate))};
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
    !std::isfinite(command.yaw_rate))
  {
    return std::nullopt;
  }
  return command;
}

std::optional<VelocityCommand> applyMinimumPlanarSpeed(
  const VelocityCommand & command,
  double minimum_speed,
  double max_vx,
  double max_vy)
{
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
    !std::isfinite(command.yaw_rate) ||
    !inClosedRange(minimum_speed, 0.0, std::min(max_vx, max_vy)) ||
    !inPositiveRange(max_vx, std::numeric_limits<float>::max()) ||
    !inPositiveRange(max_vy, std::numeric_limits<float>::max()) ||
    std::abs(command.vx) > max_vx || std::abs(command.vy) > max_vy)
  {
    return std::nullopt;
  }

  const double planar_speed = std::hypot(command.vx, command.vy);
  if (!std::isfinite(planar_speed)) {
    return std::nullopt;
  }
  if (planar_speed <= kMotionResponseCommandEpsilon || planar_speed >= minimum_speed) {
    return command;
  }

  const double scale = minimum_speed / planar_speed;
  const VelocityCommand scaled{
    static_cast<float>(static_cast<double>(command.vx) * scale),
    static_cast<float>(static_cast<double>(command.vy) * scale),
    command.yaw_rate};
  if (!std::isfinite(scaled.vx) || !std::isfinite(scaled.vy) ||
    std::abs(scaled.vx) > max_vx || std::abs(scaled.vy) > max_vy)
  {
    return std::nullopt;
  }
  return scaled;
}

void MotionResponseWatchdog::reset()
{
  mode_ = Mode::kInactive;
  checkpoint_x_ = 0.0;
  checkpoint_y_ = 0.0;
  checkpoint_yaw_ = 0.0;
  checkpoint_time_ = 0.0;
}

std::optional<bool> MotionResponseWatchdog::observe(
  const VelocityCommand & command,
  double current_x,
  double current_y,
  double current_yaw,
  double steady_time,
  double timeout,
  double minimum_translation,
  double minimum_yaw)
{
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
    !std::isfinite(command.yaw_rate) || !std::isfinite(current_x) ||
    !std::isfinite(current_y) || !std::isfinite(current_yaw) ||
    !std::isfinite(steady_time) || !inPositiveRange(timeout, 10.0) ||
    !inPositiveRange(minimum_translation, std::numeric_limits<double>::max()) ||
    !inPositiveRange(minimum_yaw, kPi))
  {
    return std::nullopt;
  }

  const double translation_command = std::hypot(command.vx, command.vy);
  const Mode next_mode = translation_command > kMotionResponseCommandEpsilon ?
    Mode::kTranslation :
    (std::abs(command.yaw_rate) > kMotionResponseCommandEpsilon ?
    Mode::kRotation : Mode::kInactive);
  if (next_mode == Mode::kInactive) {
    reset();
    return true;
  }

  if (mode_ != next_mode) {
    mode_ = next_mode;
    checkpoint_x_ = current_x;
    checkpoint_y_ = current_y;
    checkpoint_yaw_ = current_yaw;
    checkpoint_time_ = steady_time;
    return true;
  }
  if (steady_time < checkpoint_time_) {
    return std::nullopt;
  }

  const bool response_observed = mode_ == Mode::kTranslation ?
    std::hypot(current_x - checkpoint_x_, current_y - checkpoint_y_) >= minimum_translation :
    std::abs(std::remainder(current_yaw - checkpoint_yaw_, 2.0 * kPi)) >= minimum_yaw;
  if (response_observed) {
    checkpoint_x_ = current_x;
    checkpoint_y_ = current_y;
    checkpoint_yaw_ = current_yaw;
    checkpoint_time_ = steady_time;
    return true;
  }
  return steady_time - checkpoint_time_ <= timeout;
}

std::optional<bool> updateHeadingAlignmentGate(
  bool currently_active,
  double yaw_error,
  double enter_angle,
  double exit_angle)
{
  if (!std::isfinite(yaw_error) || !inPositiveRange(enter_angle, kPi) ||
    !inClosedRange(exit_angle, 0.0, enter_angle) || exit_angle >= enter_angle)
  {
    return std::nullopt;
  }

  const double absolute_error = std::abs(std::remainder(yaw_error, 2.0 * kPi));
  return currently_active ? absolute_error > exit_angle : absolute_error >= enter_angle;
}

std::optional<double> selectAlignmentYawError(
  double local_yaw_error,
  double goal_yaw_error,
  double goal_distance,
  double goal_position_tolerance,
  bool explicit_rotation_waypoint,
  bool pending_explicit_rotation)
{
  if (!std::isfinite(local_yaw_error) || !std::isfinite(goal_yaw_error) ||
    !inClosedRange(goal_distance, 0.0, std::numeric_limits<double>::max()) ||
    !inPositiveRange(goal_position_tolerance, std::numeric_limits<double>::max()))
  {
    return std::nullopt;
  }
  return !explicit_rotation_waypoint && !pending_explicit_rotation &&
         goal_distance <= goal_position_tolerance ?
         goal_yaw_error : local_yaw_error;
}

std::optional<bool> goalCompletionReady(
  double goal_distance,
  double goal_yaw_error,
  double goal_position_tolerance,
  double goal_yaw_tolerance,
  const std::optional<PathTrackingTarget> & tracking_target)
{
  if (!tracking_target ||
    !inClosedRange(goal_distance, 0.0, std::numeric_limits<double>::max()) ||
    !std::isfinite(goal_yaw_error) ||
    !inPositiveRange(goal_position_tolerance, std::numeric_limits<double>::max()) ||
    !inPositiveRange(goal_yaw_tolerance, kPi))
  {
    return std::nullopt;
  }
  return !tracking_target->pending_explicit_rotation &&
         goal_distance <= goal_position_tolerance &&
         std::abs(std::remainder(goal_yaw_error, 2.0 * kPi)) <= goal_yaw_tolerance;
}

std::optional<bool> requireRotateInPlace(
  bool hysteresis_gate_active,
  bool explicit_rotation_waypoint,
  double yaw_error,
  double explicit_rotation_tolerance,
  double goal_distance,
  double goal_position_tolerance,
  bool pending_explicit_rotation)
{
  if (!std::isfinite(yaw_error) ||
    !inClosedRange(explicit_rotation_tolerance, 0.0, kPi) ||
    !inClosedRange(goal_distance, 0.0, std::numeric_limits<double>::max()) ||
    !inPositiveRange(goal_position_tolerance, std::numeric_limits<double>::max()))
  {
    return std::nullopt;
  }
  const double absolute_error = std::abs(std::remainder(yaw_error, 2.0 * kPi));
  return hysteresis_gate_active ||
         (!pending_explicit_rotation && goal_distance <= goal_position_tolerance) ||
         (explicit_rotation_waypoint && absolute_error > explicit_rotation_tolerance);
}

std::optional<VelocityCommand> makeHeadingAwareCommand(
  double raw_vx,
  double raw_vy,
  double raw_yaw_rate,
  bool heading_alignment_active,
  double max_vx,
  double max_vy,
  double max_yaw_rate)
{
  if (!std::isfinite(raw_vx) || !std::isfinite(raw_vy) || !std::isfinite(raw_yaw_rate)) {
    return std::nullopt;
  }
  return makeBoundedCommand(
    heading_alignment_active ? 0.0 : raw_vx,
    heading_alignment_active ? 0.0 : raw_vy,
    raw_yaw_rate, max_vx, max_vy, max_yaw_rate);
}

const char * pathTrackingFailureName(PathTrackingFailure failure)
{
  switch (failure) {
    case PathTrackingFailure::kNone:
      return "none";
    case PathTrackingFailure::kInvalidInput:
      return "invalid_input";
    case PathTrackingFailure::kInvalidPose:
      return "invalid_pose";
    case PathTrackingFailure::kInvalidTrackerState:
      return "invalid_tracker_state";
    case PathTrackingFailure::kNonFiniteFinalDistance:
      return "non_finite_final_distance";
    case PathTrackingFailure::kFinalPoseTooFar:
      return "final_pose_too_far";
    case PathTrackingFailure::kNonFiniteSegmentLength:
      return "non_finite_segment_length";
    case PathTrackingFailure::kNonFiniteEdgeLength:
      return "non_finite_edge_length";
    case PathTrackingFailure::kNonFiniteWaypointDistance:
      return "non_finite_waypoint_distance";
    case PathTrackingFailure::kRotationWaypointTooFar:
      return "rotation_waypoint_too_far";
    case PathTrackingFailure::kInvalidRotationQuaternion:
      return "invalid_rotation_quaternion";
    case PathTrackingFailure::kNonFiniteYawError:
      return "non_finite_yaw_error";
    case PathTrackingFailure::kNonFiniteProgress:
      return "non_finite_progress";
    case PathTrackingFailure::kNonFiniteProjectionDistance:
      return "non_finite_projection_distance";
    case PathTrackingFailure::kProjectionTooFar:
      return "projection_too_far";
    case PathTrackingFailure::kDegenerateProgressEdge:
      return "degenerate_progress_edge";
    case PathTrackingFailure::kInvalidPlannedYaw:
      return "invalid_planned_yaw";
    case PathTrackingFailure::kNonFiniteForwardAlignment:
      return "non_finite_forward_alignment";
    case PathTrackingFailure::kIterationLimit:
      return "iteration_limit";
  }
  return "unknown";
}

void PathProgressTracker::reset()
{
  initialized_ = false;
  pose_index_ = 0U;
  segment_fraction_ = 0.0;
}

bool PathProgressTracker::reanchor(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  double current_x,
  double current_y)
{
  constexpr double same_position_tolerance = 1.0e-6;
  if (poses.empty() || !std::isfinite(current_x) || !std::isfinite(current_y) ||
    !std::all_of(
      poses.begin(), poses.end(),
      [](const geometry_msgs::msg::PoseStamped & pose) {return isFinitePose(pose.pose);}))
  {
    return false;
  }

  // There is no prior cursor to preserve on the first path of a goal. Let the
  // normal update perform its original bounded prefix search.
  if (!initialized_) {
    initialized_ = true;
    pose_index_ = 0U;
    segment_fraction_ = 0.0;
    return true;
  }

  const std::size_t previous_pose_index = pose_index_;
  const double previous_segment_fraction = segment_fraction_;
  if (poses.size() == 1U) {
    pose_index_ = 0U;
    segment_fraction_ = 0.0;
    return true;
  }

  const double refreshed_start_distance = std::hypot(
    current_x - poses.front().pose.position.x,
    current_y - poses.front().pose.position.y);
  if (!std::isfinite(refreshed_start_distance)) {
    return false;
  }

  // The flat-obstacle planner emits the exact current body pose followed by
  // a regenerated grid connector on every same-goal refresh. Its pose
  // indices therefore do not identify the same physical waypoint from one
  // refresh to the next. If the refreshed path starts at the robot, an old
  // cursor at (for example) the connector's rotation endpoint is not safe to
  // reuse: it can make update() evaluate a zero-length rotation edge before
  // the robot has reached that endpoint. Reconcile this regenerated prefix
  // from pose zero; the signed crossing checks below still prevent skipping
  // an unconfirmed segment or jumping to a future U-turn branch.
  const bool refreshed_exact_start_prefix =
    previous_pose_index > 0U &&
    refreshed_start_distance <= kShortStartConnectorReachTolerance;

  // Keep the old cursor as the lower bound. Reanchoring is intentionally not
  // a nearest-segment search: a future U-turn segment can be geometrically
  // closer than the active segment and must never be selected merely for that
  // reason.
  pose_index_ = refreshed_exact_start_prefix ? 0U :
    std::min(previous_pose_index, poses.size() - 1U);
  segment_fraction_ = refreshed_exact_start_prefix ? 0.0 :
    std::clamp(previous_segment_fraction, 0.0, 1.0);
  double advanced_length = 0.0;

  // Consume only a connector that the prior cursor had already begun or that
  // the current pose has geometrically crossed. This is deliberately narrower
  // than the commissioning cross-track policy and cannot skip a fresh route's
  // first step merely because it is short.
  for (std::size_t pass = 0U; pass < poses.size(); ++pass) {
    if (pose_index_ + 1U >= poses.size()) {
      break;
    }
    const auto & start = poses[pose_index_].pose.position;
    const auto & end = poses[pose_index_ + 1U].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length)) {
      return false;
    }
    if (length <= same_position_tolerance) {
      // A zero-length edge is an explicit rotation waypoint and must remain
      // visible to update().
      break;
    }
    const double direction_x = dx / length;
    const double direction_y = dy / length;
    const double offset_x = current_x - start.x;
    const double offset_y = current_y - start.y;
    const double progress = offset_x * direction_x + offset_y * direction_y;
    const double cross_track = std::abs(-offset_x * direction_y + offset_y * direction_x);
    if (!std::isfinite(progress) || !std::isfinite(cross_track)) {
      return false;
    }
    const double projected_fraction = std::clamp(progress / length, 0.0, 1.0);
    segment_fraction_ = pose_index_ == previous_pose_index ?
      std::max(segment_fraction_, projected_fraction) : projected_fraction;
    const bool crossed_segment =
      progress >= length - kSignedCornerReachTolerance &&
      cross_track <= kRotationWaypointTolerance;
    const bool progressed_short_connector =
      pose_index_ == 0U && previous_pose_index == 0U &&
      previous_segment_fraction > 0.5 &&
      length <= kShortStartConnectorMaxLength &&
      std::hypot(current_x - end.x, current_y - end.y) <=
      kShortStartConnectorReachTolerance;
    if (!crossed_segment && !progressed_short_connector) {
      break;
    }
    if (advanced_length + length > kMaximumPathProgressAdvance) {
      break;
    }
    ++pose_index_;
    segment_fraction_ = 0.0;
    advanced_length += length;
  }
  return true;
}

std::optional<PathTrackingTarget> PathProgressTracker::update(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  double current_x,
  double current_y,
  double current_yaw,
  double lookahead_distance,
  double explicit_rotation_tolerance,
  PathTrackingDiagnostics * diagnostics,
  bool enforce_path_cross_track_safety_gate)
{
  constexpr double same_position_tolerance = 1.0e-6;
  constexpr double same_direction_tolerance = 1.0e-6;
  constexpr double diagnostic_nan = std::numeric_limits<double>::quiet_NaN();
  if (diagnostics != nullptr) {
    *diagnostics = PathTrackingDiagnostics{};
    diagnostics->tracker_initialized = initialized_;
    diagnostics->path_pose_count = poses.size();
    diagnostics->pose_index = pose_index_;
    diagnostics->segment_fraction = segment_fraction_;
    diagnostics->current_x = current_x;
    diagnostics->current_y = current_y;
    diagnostics->current_yaw = current_yaw;
    if (!poses.empty()) {
      diagnostics->path_start_x = poses.front().pose.position.x;
      diagnostics->path_start_y = poses.front().pose.position.y;
      diagnostics->path_final_x = poses.back().pose.position.x;
      diagnostics->path_final_y = poses.back().pose.position.y;
    }
  }
  const auto sync_cursor_diagnostics = [this, diagnostics]() {
      if (diagnostics != nullptr) {
        diagnostics->tracker_initialized = initialized_;
        diagnostics->pose_index = pose_index_;
        diagnostics->segment_fraction = segment_fraction_;
      }
    };
  const auto fail = [&sync_cursor_diagnostics, diagnostics](PathTrackingFailure failure) {
      sync_cursor_diagnostics();
      if (diagnostics != nullptr) {
        diagnostics->failure = failure;
      }
      return std::optional<PathTrackingTarget>{};
    };
  if (poses.empty() || !std::isfinite(current_x) || !std::isfinite(current_y) ||
    !std::isfinite(current_yaw) ||
    !inPositiveRange(lookahead_distance, std::numeric_limits<double>::max()) ||
    !inClosedRange(explicit_rotation_tolerance, 0.0, kPi))
  {
    return fail(PathTrackingFailure::kInvalidInput);
  }
  if (!std::all_of(
      poses.begin(), poses.end(),
      [](const geometry_msgs::msg::PoseStamped & pose) {return isFinitePose(pose.pose);}))
  {
    return fail(PathTrackingFailure::kInvalidPose);
  }

  if (!initialized_) {
    initialized_ = true;
    pose_index_ = 0U;
    segment_fraction_ = 0.0;
  }
  if (pose_index_ >= poses.size() || !inClosedRange(segment_fraction_, 0.0, 1.0)) {
    return fail(PathTrackingFailure::kInvalidTrackerState);
  }
  sync_cursor_diagnostics();

  const auto has_pending_rotation = [&poses](std::size_t first_pose) {
      for (std::size_t index = first_pose; index + 1U < poses.size(); ++index) {
        const auto & first = poses[index].pose.position;
        const auto & second = poses[index + 1U].pose.position;
        if (std::hypot(second.x - first.x, second.y - first.y) <=
          same_position_tolerance)
        {
          return true;
        }
      }
      return false;
    };

  // Each pass either consumes one reached corner/rotation or returns a target.
  // The bound prevents malformed zero-length paths from looping indefinitely.
  for (std::size_t pass = 0U; pass <= poses.size(); ++pass) {
    if (diagnostics != nullptr) {
      diagnostics->block_end = pose_index_;
      diagnostics->segment_start_x = diagnostic_nan;
      diagnostics->segment_start_y = diagnostic_nan;
      diagnostics->segment_end_x = diagnostic_nan;
      diagnostics->segment_end_y = diagnostic_nan;
      diagnostics->segment_length = diagnostic_nan;
      diagnostics->block_length = diagnostic_nan;
      diagnostics->previous_progress = diagnostic_nan;
      diagnostics->current_progress = diagnostic_nan;
      diagnostics->maximum_progress = diagnostic_nan;
      diagnostics->projected_progress = diagnostic_nan;
      diagnostics->cross_track = diagnostic_nan;
      diagnostics->projection_distance = diagnostic_nan;
      diagnostics->waypoint_distance = diagnostic_nan;
      diagnostics->final_distance = diagnostic_nan;
      diagnostics->forward_alignment = diagnostic_nan;
    }
    sync_cursor_diagnostics();
    if (pose_index_ + 1U >= poses.size()) {
      const auto & final = poses.back().pose.position;
      const double final_distance = std::hypot(current_x - final.x, current_y - final.y);
      if (diagnostics != nullptr) {
        diagnostics->final_distance = final_distance;
      }
      if (!std::isfinite(final_distance) ||
        (enforce_path_cross_track_safety_gate &&
        final_distance > kMaximumPathCrossTrack))
      {
        return fail(
          !std::isfinite(final_distance) ?
          PathTrackingFailure::kNonFiniteFinalDistance :
          PathTrackingFailure::kFinalPoseTooFar);
      }
      return PathTrackingTarget{
        final.x, final.y, pose_index_, segment_fraction_, poses.size() - 1U,
        false, false, PlannedTranslationDirection::kForward};
    }

    const auto & start = poses[pose_index_].pose.position;
    const auto & next = poses[pose_index_ + 1U].pose.position;
    const double first_dx = next.x - start.x;
    const double first_dy = next.y - start.y;
    const double first_length = std::hypot(first_dx, first_dy);
    if (diagnostics != nullptr) {
      diagnostics->segment_start_x = start.x;
      diagnostics->segment_start_y = start.y;
      diagnostics->segment_end_x = next.x;
      diagnostics->segment_end_y = next.y;
      diagnostics->segment_length = first_length;
    }
    if (!std::isfinite(first_length)) {
      return fail(PathTrackingFailure::kNonFiniteSegmentLength);
    }

    if (first_length <= same_position_tolerance) {
      const double waypoint_distance = std::hypot(
        current_x - start.x, current_y - start.y);
      if (diagnostics != nullptr) {
        diagnostics->waypoint_distance = waypoint_distance;
      }
      if (!std::isfinite(waypoint_distance) ||
        (enforce_path_cross_track_safety_gate &&
        waypoint_distance > kRotationWaypointTolerance))
      {
        return fail(
          !std::isfinite(waypoint_distance) ?
          PathTrackingFailure::kNonFiniteWaypointDistance :
          PathTrackingFailure::kRotationWaypointTooFar);
      }
      const auto target_yaw = quaternionYaw(
        poses[pose_index_ + 1U].pose.orientation);
      if (!target_yaw) {
        return fail(PathTrackingFailure::kInvalidRotationQuaternion);
      }
      const double yaw_error = std::remainder(*target_yaw - current_yaw, 2.0 * kPi);
      if (!std::isfinite(yaw_error)) {
        return fail(PathTrackingFailure::kNonFiniteYawError);
      }
      if (std::abs(yaw_error) > explicit_rotation_tolerance) {
        return PathTrackingTarget{
          start.x, start.y, pose_index_, 0.0, pose_index_ + 1U, true,
          true, PlannedTranslationDirection::kForward};
      }
      ++pose_index_;
      segment_fraction_ = 0.0;
      continue;
    }

    const double direction_x = first_dx / first_length;
    const double direction_y = first_dy / first_length;
    std::size_t block_end = pose_index_ + 1U;
    double block_length = first_length;
    while (block_end + 1U < poses.size()) {
      const auto & edge_start = poses[block_end].pose.position;
      const auto & edge_end = poses[block_end + 1U].pose.position;
      const double edge_dx = edge_end.x - edge_start.x;
      const double edge_dy = edge_end.y - edge_start.y;
      const double edge_length = std::hypot(edge_dx, edge_dy);
      if (!std::isfinite(edge_length)) {
        if (diagnostics != nullptr) {
          diagnostics->block_end = block_end;
          diagnostics->segment_start_x = edge_start.x;
          diagnostics->segment_start_y = edge_start.y;
          diagnostics->segment_end_x = edge_end.x;
          diagnostics->segment_end_y = edge_end.y;
          diagnostics->segment_length = edge_length;
          diagnostics->block_length = block_length;
        }
        return fail(PathTrackingFailure::kNonFiniteEdgeLength);
      }
      if (edge_length <= same_position_tolerance) {
        break;
      }
      const double direction_dot =
        direction_x * edge_dx / edge_length + direction_y * edge_dy / edge_length;
      if (direction_dot < 1.0 - same_direction_tolerance) {
        break;
      }
      block_length += edge_length;
      ++block_end;
    }

    const double previous_progress = segment_fraction_ * first_length;
    const double current_offset_x = current_x - start.x;
    const double current_offset_y = current_y - start.y;
    const double current_progress =
      current_offset_x * direction_x + current_offset_y * direction_y;
    const double cross_track = std::abs(
      -current_offset_x * direction_y + current_offset_y * direction_x);
    if (diagnostics != nullptr) {
      diagnostics->block_end = block_end;
      diagnostics->block_length = block_length;
      diagnostics->previous_progress = previous_progress;
      diagnostics->current_progress = current_progress;
      diagnostics->cross_track = cross_track;
    }
    if (!std::isfinite(current_progress) || !std::isfinite(cross_track)) {
      return fail(PathTrackingFailure::kNonFiniteProgress);
    }

    const double maximum_progress = std::min(
      block_length, previous_progress + kMaximumPathProgressAdvance);
    const double projected_progress = std::clamp(
      current_progress, previous_progress, maximum_progress);
    const double projected_x = start.x + direction_x * projected_progress;
    const double projected_y = start.y + direction_y * projected_progress;
    const double projection_distance = std::hypot(
      current_x - projected_x, current_y - projected_y);
    if (diagnostics != nullptr) {
      diagnostics->maximum_progress = maximum_progress;
      diagnostics->projected_progress = projected_progress;
      diagnostics->projection_distance = projection_distance;
    }
    if (!std::isfinite(projection_distance) ||
      (enforce_path_cross_track_safety_gate &&
      projection_distance > kMaximumPathCrossTrack))
    {
      return fail(
        !std::isfinite(projection_distance) ?
        PathTrackingFailure::kNonFiniteProjectionDistance :
        PathTrackingFailure::kProjectionTooFar);
    }

    const bool reached_block_end =
      block_length <= maximum_progress + same_position_tolerance &&
      current_progress >= block_length - kSignedCornerReachTolerance &&
      cross_track <= kRotationWaypointTolerance;
    if (reached_block_end) {
      pose_index_ = block_end;
      segment_fraction_ = 0.0;
      continue;
    }

    std::size_t progress_pose = pose_index_;
    double progress_on_edge = projected_progress;
    while (progress_pose + 1U < block_end) {
      const auto & edge_start = poses[progress_pose].pose.position;
      const auto & edge_end = poses[progress_pose + 1U].pose.position;
      const double edge_length = std::hypot(
        edge_end.x - edge_start.x, edge_end.y - edge_start.y);
      if (progress_on_edge < edge_length - same_position_tolerance) {
        break;
      }
      progress_on_edge -= edge_length;
      ++progress_pose;
    }
    const auto & progress_start = poses[progress_pose].pose.position;
    const auto & progress_end = poses[progress_pose + 1U].pose.position;
    const double progress_edge_length = std::hypot(
      progress_end.x - progress_start.x, progress_end.y - progress_start.y);
    if (progress_edge_length <= same_position_tolerance) {
      return fail(PathTrackingFailure::kDegenerateProgressEdge);
    }
    pose_index_ = progress_pose;
    segment_fraction_ = std::clamp(progress_on_edge / progress_edge_length, 0.0, 1.0);
    sync_cursor_diagnostics();

    const double target_progress = std::min(
      projected_progress + lookahead_distance, block_length);
    const auto planned_yaw = quaternionYaw(poses[pose_index_].pose.orientation);
    if (!planned_yaw) {
      return fail(PathTrackingFailure::kInvalidPlannedYaw);
    }
    const double forward_alignment =
      std::cos(*planned_yaw) * direction_x + std::sin(*planned_yaw) * direction_y;
    if (!std::isfinite(forward_alignment)) {
      return fail(PathTrackingFailure::kNonFiniteForwardAlignment);
    }
    if (diagnostics != nullptr) {
      diagnostics->forward_alignment = forward_alignment;
    }
    const PlannedTranslationDirection translation_direction =
      std::abs(forward_alignment) <= kLateralForwardAlignmentTolerance ?
      PlannedTranslationDirection::kLateral :
      (forward_alignment < 0.0 ? PlannedTranslationDirection::kReverse :
      PlannedTranslationDirection::kForward);
    return PathTrackingTarget{
      start.x + direction_x * target_progress,
      start.y + direction_y * target_progress,
      pose_index_, segment_fraction_, pose_index_, false,
      has_pending_rotation(pose_index_),
      translation_direction};
  }
  return fail(PathTrackingFailure::kIterationLimit);
}

std::optional<double> filterLongitudinalCommand(
  double raw_vx, PlannedTranslationDirection translation_direction, double tolerance)
{
  if (!std::isfinite(raw_vx) || !std::isfinite(tolerance) || tolerance < 0.0) {
    return std::nullopt;
  }
  switch (translation_direction) {
    case PlannedTranslationDirection::kLateral:
      return 0.0;
    case PlannedTranslationDirection::kReverse:
      return raw_vx;
    case PlannedTranslationDirection::kForward:
      if (raw_vx < -tolerance) {
        return std::nullopt;
      }
      return std::max(0.0, raw_vx);
  }
  return std::nullopt;
}

std::optional<std::int64_t> pathGoalGeneration(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (poses.empty()) {
    return std::nullopt;
  }

  const auto generation_for = [](const geometry_msgs::msg::PoseStamped & pose)
    -> std::optional<std::int64_t>
    {
      const auto & stamp = pose.header.stamp;
      if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
        (stamp.sec == 0 && stamp.nanosec == 0U))
      {
        return std::nullopt;
      }
      return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
             static_cast<std::int64_t>(stamp.nanosec);
    };

  const auto generation = generation_for(poses.front());
  if (!generation) {
    return std::nullopt;
  }
  for (const auto & pose : poses) {
    const auto candidate = generation_for(pose);
    if (!candidate || *candidate != *generation) {
      return std::nullopt;
    }
  }
  return generation;
}

void CompletedGoalLatch::markCompleted(std::int64_t goal_generation)
{
  latest_generation_ = goal_generation;
  completed_generation_ = goal_generation;
}

GoalGenerationDecision CompletedGoalLatch::evaluate(std::int64_t candidate_generation)
{
  if (candidate_generation <= 0) {
    return GoalGenerationDecision::kInvalid;
  }
  if (!latest_generation_ || candidate_generation > *latest_generation_) {
    latest_generation_ = candidate_generation;
    completed_generation_.reset();
    return GoalGenerationDecision::kAccept;
  }
  if (candidate_generation < *latest_generation_) {
    return GoalGenerationDecision::kSuperseded;
  }
  return completed_generation_ && candidate_generation == *completed_generation_ ?
         GoalGenerationDecision::kCompletedReplay : GoalGenerationDecision::kAccept;
}

void CompletedGoalLatch::clear()
{
  latest_generation_.reset();
  completed_generation_.reset();
}

bool CompletedGoalLatch::active() const
{
  return completed_generation_.has_value();
}

}  // namespace utree_go2_sdk2_bridge
