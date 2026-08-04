#pragma once

#include <optional>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace utree_go2_sdk2_bridge
{

struct ControlParameters
{
  double command_rate;
  double path_timeout;
  double odom_timeout;
  double timestamp_future_tolerance;
  double lookahead_distance;
  double goal_position_tolerance;
  double goal_yaw_tolerance;
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

}  // namespace utree_go2_sdk2_bridge
