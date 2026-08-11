#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace utree_go2_sdk2_bridge
{

inline constexpr double kValidatedMaxVx = 0.1;
inline constexpr double kValidatedMaxVy = 0.05;
inline constexpr double kValidatedMaxYawRate = 0.2;
inline constexpr double kRotationWaypointTolerance = 0.05;
inline constexpr double kMaximumPathProgressAdvance = 0.4;
inline constexpr double kMaximumPathCrossTrack = 0.05;
inline constexpr double kSignedCornerReachTolerance = 1.0e-3;
inline constexpr double kUnexpectedReverseTolerance = 1.0e-4;

struct ControlParameters
{
  double command_rate;
  double path_timeout;
  double odom_timeout;
  double timestamp_future_tolerance;
  double lookahead_distance;
  double goal_position_tolerance;
  double goal_yaw_tolerance;
  double heading_alignment_enter_angle;
  double heading_alignment_exit_angle;
  double linear_gain;
  double yaw_gain;
  double max_vx;
  double max_vy;
  double max_yaw_rate;
};

struct VelocityCommand
{
  float vx;
  float vy;
  float yaw_rate;
};

struct PathTrackingTarget
{
  double target_x;
  double target_y;
  std::size_t progress_pose;
  double progress_fraction;
  std::size_t heading_pose;
  bool explicit_rotation_waypoint;
  bool reverse_motion;
};

// Maintains bounded, monotonic progress on one Path message. Reset it whenever
// a new Path is accepted; the first update searches only the path prefix.
class PathProgressTracker
{
public:
  void reset();

  std::optional<PathTrackingTarget> update(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    double current_x,
    double current_y,
    double current_yaw,
    double lookahead_distance,
    double heading_alignment_tolerance);

private:
  bool initialized_{false};
  std::size_t pose_index_{0U};
  double segment_fraction_{0.0};
};

class CompletedGoalLatch
{
public:
  void markCompleted(std::int64_t goal_generation);
  bool accept(std::int64_t candidate_generation);
  void clear();
  bool active() const;

private:
  std::optional<std::int64_t> latest_generation_;
  std::optional<std::int64_t> completed_generation_;
};

enum class MotionAuthorizationState
{
  kDisarmed,
  kArmedWaitingForPath,
  kArmedExecuting,
};

// Separates a one-time operator arm from the presence of a currently executable path.
class MotionAuthorization
{
public:
  MotionAuthorizationState state() const;
  bool armed() const;
  bool executionAuthorized() const;

  void arm(bool path_available);
  void pathAvailable();
  void waitForPath();
  void disarm();

private:
  MotionAuthorizationState state_{MotionAuthorizationState::kDisarmed};
};

// Returns an empty string when every parameter is inside the bridge's hard safety envelope.
std::string validateControlParameters(const ControlParameters & parameters);

bool isFinitePose(const geometry_msgs::msg::Pose & pose);

// A valid ROS quaternion is finite, non-zero, and approximately unit length.
bool isValidQuaternion(const geometry_msgs::msg::Quaternion & quaternion);

std::optional<double> quaternionYaw(const geometry_msgs::msg::Quaternion & quaternion);

// Checks raw values before clamp, validates clamp bounds, and checks the float result.
std::optional<VelocityCommand> makeBoundedCommand(
  double raw_vx,
  double raw_vy,
  double raw_yaw_rate,
  double max_vx,
  double max_vy,
  double max_yaw_rate);

std::optional<bool> updateHeadingAlignmentGate(
  bool currently_active,
  double yaw_error,
  double enter_angle,
  double exit_angle);

std::optional<double> selectAlignmentYawError(
  double local_yaw_error,
  double goal_yaw_error,
  double goal_distance,
  double goal_position_tolerance);

std::optional<bool> requireRotateInPlace(
  bool hysteresis_gate_active,
  bool explicit_rotation_waypoint,
  double yaw_error,
  double alignment_exit_angle,
  double goal_distance,
  double goal_position_tolerance);

std::optional<VelocityCommand> makeHeadingAwareCommand(
  double raw_vx,
  double raw_vy,
  double raw_yaw_rate,
  bool heading_alignment_active,
  double max_vx,
  double max_vy,
  double max_yaw_rate);

// A path segment whose tangent points behind its planned body yaw is an
// explicit reverse primitive. Every other segment must emit non-negative vx.
std::optional<double> rejectUnexpectedReverseCommand(
  double raw_vx,
  bool reverse_motion,
  double tolerance = kUnexpectedReverseTolerance);

// Every pose in one non-empty Path carries the originating /goal_pose stamp.
// A single stable generation lets the bridge distinguish a planner refresh
// from an explicit new operator goal without changing Path.header freshness.
std::optional<std::int64_t> pathGoalGeneration(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses);

}  // namespace utree_go2_sdk2_bridge
