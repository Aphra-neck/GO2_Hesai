#pragma once

#include <string>

#include "nav_msgs/msg/odometry.hpp"

namespace utree_dog_navigation
{

enum class BodyOdometryStatus
{
  kOk,
  kInvalidConfiguration,
  kFrameMismatch,
  kInvalidPose,
  kInvalidTwist,
};

const char * bodyOdometryStatusMessage(BodyOdometryStatus status);

// Converts Super-LIO's world-frame IMU odometry into a body-frame odometry pose.
BodyOdometryStatus makeBodyOdometry(
  const nav_msgs::msg::Odometry & source,
  double yaw_offset,
  const std::string & world_frame,
  const std::string & body_frame,
  nav_msgs::msg::Odometry & output);

}  // namespace utree_dog_navigation
