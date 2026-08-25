#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace utree_go2_sdk2_bridge
{

// max_vx is a symmetric magnitude, so the SDK's -2.5 m/s reverse boundary is
// the limiting side even though its forward boundary is +3.8 m/s.
inline constexpr double kSdkMaxSymmetricVx = 2.5;
inline constexpr double kSdkMaxAbsVy = 1.0;
inline constexpr double kSdkMaxAbsYawRate = 4.0;
inline constexpr double kRotationWaypointTolerance = 0.05;
inline constexpr double kMaximumPathProgressAdvance = 0.4;
// The bridge may disable this commissioning-time geometric rejection gate
// while retaining all finite-value and command-direction checks.
inline constexpr double kMaximumPathCrossTrack = 0.05;
inline constexpr double kSignedCornerReachTolerance = 1.0e-3;
inline constexpr double kUnexpectedReverseTolerance = 1.0e-4;
inline constexpr double kMotionResponseCommandEpsilon = 1.0e-4;
// The 16-bin planner's closest rounded diagonal is 22.5 degrees from lateral.
// Split that interval at 11.25 degrees so grid jitter stays lateral while the
// diagonal primitive keeps its longitudinal component: sin(pi / 16).
inline constexpr double kLateralForwardAlignmentTolerance = 0.19509032201612825;

struct ControlParameters
{
  double command_rate;
  double path_timeout;
  double odom_timeout;
  double sport_state_timeout;
  double timestamp_future_tolerance;
  double lookahead_distance;
  double goal_position_tolerance;
  double goal_yaw_tolerance;
  double heading_alignment_enter_angle;
  double heading_alignment_exit_angle;
  double explicit_rotation_tolerance;
  double linear_gain;
  double yaw_gain;
  double minimum_translation_speed;
  double motion_response_timeout;
  double motion_response_min_translation;
  double motion_response_min_yaw;
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

// Raises a non-zero planar command above the empirically observed no-response
// band while preserving its direction and the already bounded yaw command.
std::optional<VelocityCommand> applyMinimumPlanarSpeed(
  const VelocityCommand & command,
  double minimum_speed,
  double max_vx,
  double max_vy);

// Fails closed when a continuing non-zero command produces no bounded odometry
// progress. The steady_time argument keeps this helper independent of ROS time.
class MotionResponseWatchdog
{
public:
  void reset();

  std::optional<bool> observe(
    const VelocityCommand & command,
    double current_x,
    double current_y,
    double current_yaw,
    double steady_time,
    double timeout,
    double minimum_translation,
    double minimum_yaw);

private:
  enum class Mode
  {
    kInactive,
    kTranslation,
    kRotation,
  };

  Mode mode_{Mode::kInactive};
  double checkpoint_x_{0.0};
  double checkpoint_y_{0.0};
  double checkpoint_yaw_{0.0};
  double checkpoint_time_{0.0};
};

enum class PlannedTranslationDirection
{
  kForward,
  kLateral,
  kReverse,
};

struct PathTrackingTarget
{
  double target_x;
  double target_y;
  std::size_t progress_pose;
  double progress_fraction;
  std::size_t heading_pose;
  bool explicit_rotation_waypoint;
  bool pending_explicit_rotation;
  PlannedTranslationDirection translation_direction;
};

enum class PathTrackingFailure
{
  kNone,
  kInvalidInput,
  kInvalidPose,
  kInvalidTrackerState,
  kNonFiniteFinalDistance,
  kFinalPoseTooFar,
  kNonFiniteSegmentLength,
  kNonFiniteEdgeLength,
  kNonFiniteWaypointDistance,
  kRotationWaypointTooFar,
  kInvalidRotationQuaternion,
  kNonFiniteYawError,
  kNonFiniteProgress,
  kNonFiniteProjectionDistance,
  kProjectionTooFar,
  kDegenerateProgressEdge,
  kInvalidPlannedYaw,
  kNonFiniteForwardAlignment,
  kIterationLimit,
};

const char * pathTrackingFailureName(PathTrackingFailure failure);

// Captures the exact fail-closed branch and the geometric values available at
// that branch. It is diagnostic-only and does not participate in decisions.
struct PathTrackingDiagnostics
{
  PathTrackingFailure failure{PathTrackingFailure::kNone};
  bool tracker_initialized{false};
  std::size_t path_pose_count{0U};
  std::size_t pose_index{0U};
  std::size_t block_end{0U};
  double segment_fraction{0.0};
  double current_x{std::numeric_limits<double>::quiet_NaN()};
  double current_y{std::numeric_limits<double>::quiet_NaN()};
  double current_yaw{std::numeric_limits<double>::quiet_NaN()};
  double path_start_x{std::numeric_limits<double>::quiet_NaN()};
  double path_start_y{std::numeric_limits<double>::quiet_NaN()};
  double path_final_x{std::numeric_limits<double>::quiet_NaN()};
  double path_final_y{std::numeric_limits<double>::quiet_NaN()};
  double segment_start_x{std::numeric_limits<double>::quiet_NaN()};
  double segment_start_y{std::numeric_limits<double>::quiet_NaN()};
  double segment_end_x{std::numeric_limits<double>::quiet_NaN()};
  double segment_end_y{std::numeric_limits<double>::quiet_NaN()};
  double segment_length{std::numeric_limits<double>::quiet_NaN()};
  double block_length{std::numeric_limits<double>::quiet_NaN()};
  double previous_progress{std::numeric_limits<double>::quiet_NaN()};
  double current_progress{std::numeric_limits<double>::quiet_NaN()};
  double maximum_progress{std::numeric_limits<double>::quiet_NaN()};
  double projected_progress{std::numeric_limits<double>::quiet_NaN()};
  double cross_track{std::numeric_limits<double>::quiet_NaN()};
  double projection_distance{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_distance{std::numeric_limits<double>::quiet_NaN()};
  double final_distance{std::numeric_limits<double>::quiet_NaN()};
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
    double explicit_rotation_tolerance,
    PathTrackingDiagnostics * diagnostics = nullptr,
    bool enforce_path_cross_track_safety_gate = true);

private:
  bool initialized_{false};
  std::size_t pose_index_{0U};
  double segment_fraction_{0.0};
};

enum class GoalGenerationDecision
{
  kAccept,
  kCompletedReplay,
  kSuperseded,
  kInvalid,
};

struct SportStateSample
{
  std::uint32_t state_code;
  std::uint8_t mode;
  std::uint8_t gait_type;
};

// The deployed firmware reports the high-level motion state through the
// historical error_code field of rt/sportmodestate.
bool isExecutableSportState(std::uint32_t state_code);

const char * sportStateName(std::uint32_t state_code);

// Preserve an unsafe transient until the ROS control loop can fail closed.
class UnsafeSportStateLatch
{
public:
  void observe(const SportStateSample & sample);
  std::optional<SportStateSample> take();

private:
  std::optional<SportStateSample> pending_sample_;
};

class CompletedGoalLatch
{
public:
  void markCompleted(std::int64_t goal_generation);
  GoalGenerationDecision evaluate(std::int64_t candidate_generation);
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
  double goal_position_tolerance,
  bool explicit_rotation_waypoint = false,
  bool pending_explicit_rotation = false);

std::optional<bool> goalCompletionReady(
  double goal_distance,
  double goal_yaw_error,
  double goal_position_tolerance,
  double goal_yaw_tolerance,
  const std::optional<PathTrackingTarget> & tracking_target);

std::optional<bool> requireRotateInPlace(
  bool hysteresis_gate_active,
  bool explicit_rotation_waypoint,
  double yaw_error,
  double explicit_rotation_tolerance,
  double goal_distance,
  double goal_position_tolerance,
  bool pending_explicit_rotation = false);

std::optional<VelocityCommand> makeHeadingAwareCommand(
  double raw_vx,
  double raw_vy,
  double raw_yaw_rate,
  bool heading_alignment_active,
  double max_vx,
  double max_vy,
  double max_yaw_rate);

// Lateral primitives emit no longitudinal command. Only an explicit reverse
// primitive may emit negative vx; forward primitives retain the fail-closed gate.
std::optional<double> filterLongitudinalCommand(
  double raw_vx,
  PlannedTranslationDirection translation_direction,
  double tolerance = kUnexpectedReverseTolerance);

// Every pose in one non-empty Path carries the originating /goal_pose stamp.
// A single stable generation lets the bridge distinguish a planner refresh
// from an explicit new operator goal without changing Path.header freshness.
std::optional<std::int64_t> pathGoalGeneration(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses);

}  // namespace utree_go2_sdk2_bridge
