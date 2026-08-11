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
  // These are hard rejection limits for the validated flat-ground stage.
  if (!inClosedRange(parameters.command_rate, 1.0, 200.0)) {
    return "command_rate must be finite and in [1, 200] Hz";
  }
  if (!inPositiveRange(parameters.path_timeout, 60.0)) {
    return "path_timeout must be finite and in (0, 60] seconds";
  }
  if (!inPositiveRange(parameters.odom_timeout, 60.0)) {
    return "odom_timeout must be finite and in (0, 60] seconds";
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
  if (!inClosedRange(parameters.linear_gain, 0.0, 20.0)) {
    return "linear_gain must be finite and in [0, 20]";
  }
  if (!inClosedRange(parameters.yaw_gain, 0.0, 20.0)) {
    return "yaw_gain must be finite and in [0, 20]";
  }
  if (!inPositiveRange(parameters.max_vx, kValidatedMaxVx)) {
    return "max_vx must be finite and in (0, 0.1] m/s";
  }
  if (!inPositiveRange(parameters.max_vy, kValidatedMaxVy)) {
    return "max_vy must be finite and in (0, 0.05] m/s";
  }
  if (!inPositiveRange(parameters.max_yaw_rate, kValidatedMaxYawRate)) {
    return "max_yaw_rate must be finite and in (0, 0.2] rad/s";
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
  double goal_position_tolerance)
{
  if (!std::isfinite(local_yaw_error) || !std::isfinite(goal_yaw_error) ||
    !inClosedRange(goal_distance, 0.0, std::numeric_limits<double>::max()) ||
    !inPositiveRange(goal_position_tolerance, std::numeric_limits<double>::max()))
  {
    return std::nullopt;
  }
  return goal_distance <= goal_position_tolerance ? goal_yaw_error : local_yaw_error;
}

std::optional<bool> requireRotateInPlace(
  bool hysteresis_gate_active,
  bool explicit_rotation_waypoint,
  double yaw_error,
  double alignment_exit_angle,
  double goal_distance,
  double goal_position_tolerance)
{
  if (!std::isfinite(yaw_error) ||
    !inClosedRange(alignment_exit_angle, 0.0, kPi) ||
    !inClosedRange(goal_distance, 0.0, std::numeric_limits<double>::max()) ||
    !inPositiveRange(goal_position_tolerance, std::numeric_limits<double>::max()))
  {
    return std::nullopt;
  }
  const double absolute_error = std::abs(std::remainder(yaw_error, 2.0 * kPi));
  return hysteresis_gate_active || goal_distance <= goal_position_tolerance ||
         (explicit_rotation_waypoint && absolute_error > alignment_exit_angle);
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

void PathProgressTracker::reset()
{
  initialized_ = false;
  pose_index_ = 0U;
  segment_fraction_ = 0.0;
}

std::optional<PathTrackingTarget> PathProgressTracker::update(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  double current_x,
  double current_y,
  double current_yaw,
  double lookahead_distance,
  double heading_alignment_tolerance)
{
  constexpr double same_position_tolerance = 1.0e-6;
  constexpr double same_direction_tolerance = 1.0e-6;
  if (poses.empty() || !std::isfinite(current_x) || !std::isfinite(current_y) ||
    !std::isfinite(current_yaw) ||
    !inPositiveRange(lookahead_distance, std::numeric_limits<double>::max()) ||
    !inClosedRange(heading_alignment_tolerance, 0.0, kPi))
  {
    return std::nullopt;
  }
  if (!std::all_of(
      poses.begin(), poses.end(),
      [](const geometry_msgs::msg::PoseStamped & pose) {return isFinitePose(pose.pose);}))
  {
    return std::nullopt;
  }

  if (!initialized_) {
    initialized_ = true;
    pose_index_ = 0U;
    segment_fraction_ = 0.0;
  }
  if (pose_index_ >= poses.size() || !inClosedRange(segment_fraction_, 0.0, 1.0)) {
    return std::nullopt;
  }

  // Each pass either consumes one reached corner/rotation or returns a target.
  // The bound prevents malformed zero-length paths from looping indefinitely.
  for (std::size_t pass = 0U; pass <= poses.size(); ++pass) {
    if (pose_index_ + 1U >= poses.size()) {
      const auto & final = poses.back().pose.position;
      const double final_distance = std::hypot(current_x - final.x, current_y - final.y);
      if (!std::isfinite(final_distance) || final_distance > kMaximumPathCrossTrack) {
        return std::nullopt;
      }
      return PathTrackingTarget{
        final.x, final.y, pose_index_, segment_fraction_, poses.size() - 1U,
        false, false};
    }

    const auto & start = poses[pose_index_].pose.position;
    const auto & next = poses[pose_index_ + 1U].pose.position;
    const double first_dx = next.x - start.x;
    const double first_dy = next.y - start.y;
    const double first_length = std::hypot(first_dx, first_dy);
    if (!std::isfinite(first_length)) {
      return std::nullopt;
    }

    if (first_length <= same_position_tolerance) {
      const double waypoint_distance = std::hypot(
        current_x - start.x, current_y - start.y);
      if (!std::isfinite(waypoint_distance) ||
        waypoint_distance > kRotationWaypointTolerance)
      {
        return std::nullopt;
      }
      const auto target_yaw = quaternionYaw(
        poses[pose_index_ + 1U].pose.orientation);
      if (!target_yaw) {
        return std::nullopt;
      }
      const double yaw_error = std::remainder(*target_yaw - current_yaw, 2.0 * kPi);
      if (!std::isfinite(yaw_error)) {
        return std::nullopt;
      }
      if (std::abs(yaw_error) > heading_alignment_tolerance) {
        return PathTrackingTarget{
          start.x, start.y, pose_index_, 0.0, pose_index_ + 1U, true, false};
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
        return std::nullopt;
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
    if (!std::isfinite(current_progress) || !std::isfinite(cross_track)) {
      return std::nullopt;
    }

    const double maximum_progress = std::min(
      block_length, previous_progress + kMaximumPathProgressAdvance);
    const double projected_progress = std::clamp(
      current_progress, previous_progress, maximum_progress);
    const double projected_x = start.x + direction_x * projected_progress;
    const double projected_y = start.y + direction_y * projected_progress;
    const double projection_distance = std::hypot(
      current_x - projected_x, current_y - projected_y);
    if (!std::isfinite(projection_distance) ||
      projection_distance > kMaximumPathCrossTrack)
    {
      return std::nullopt;
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
      return std::nullopt;
    }
    pose_index_ = progress_pose;
    segment_fraction_ = std::clamp(progress_on_edge / progress_edge_length, 0.0, 1.0);

    const double target_progress = std::min(
      projected_progress + lookahead_distance, block_length);
    const auto planned_yaw = quaternionYaw(poses[pose_index_].pose.orientation);
    if (!planned_yaw) {
      return std::nullopt;
    }
    const double forward_alignment =
      std::cos(*planned_yaw) * direction_x + std::sin(*planned_yaw) * direction_y;
    if (!std::isfinite(forward_alignment)) {
      return std::nullopt;
    }
    return PathTrackingTarget{
      start.x + direction_x * target_progress,
      start.y + direction_y * target_progress,
      pose_index_, segment_fraction_, pose_index_, false,
      forward_alignment < -same_direction_tolerance};
  }
  return std::nullopt;
}

std::optional<double> rejectUnexpectedReverseCommand(
  double raw_vx, bool reverse_motion, double tolerance)
{
  if (!std::isfinite(raw_vx) || !std::isfinite(tolerance) || tolerance < 0.0) {
    return std::nullopt;
  }
  if (reverse_motion) {
    return raw_vx;
  }
  if (raw_vx < -tolerance) {
    return std::nullopt;
  }
  return std::max(0.0, raw_vx);
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

bool CompletedGoalLatch::accept(std::int64_t candidate_generation)
{
  if (candidate_generation <= 0) {
    return false;
  }
  if (!latest_generation_ || candidate_generation > *latest_generation_) {
    latest_generation_ = candidate_generation;
    completed_generation_.reset();
    return true;
  }
  if (candidate_generation < *latest_generation_) {
    return false;
  }
  return !completed_generation_ || candidate_generation != *completed_generation_;
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
