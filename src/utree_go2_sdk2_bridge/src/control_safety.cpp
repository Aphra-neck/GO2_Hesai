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

bool inClosedRange(double value, double minimum, double maximum)
{
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

bool inPositiveRange(double value, double maximum)
{
  return std::isfinite(value) && value > 0.0 && value <= maximum;
}
}  // namespace

std::string validateControlParameters(const ControlParameters & parameters)
{
  // These are hard rejection limits, not operating recommendations. The shipped
  // configuration remains substantially below the velocity limits.
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
  if (!inClosedRange(parameters.linear_gain, 0.0, 20.0)) {
    return "linear_gain must be finite and in [0, 20]";
  }
  if (!inClosedRange(parameters.yaw_gain, 0.0, 20.0)) {
    return "yaw_gain must be finite and in [0, 20]";
  }
  if (!inPositiveRange(parameters.max_vx, 2.5)) {
    return "max_vx must be finite and in (0, 2.5] m/s";
  }
  if (!inPositiveRange(parameters.max_vy, 1.0)) {
    return "max_vy must be finite and in (0, 1] m/s";
  }
  if (!inPositiveRange(parameters.max_yaw_rate, 4.0)) {
    return "max_yaw_rate must be finite and in (0, 4] rad/s";
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

}  // namespace utree_go2_sdk2_bridge
