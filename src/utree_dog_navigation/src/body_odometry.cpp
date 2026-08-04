#include "utree_dog_navigation/body_odometry.hpp"

#include <array>
#include <cmath>
#include <utility>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace utree_dog_navigation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kQuaternionNormEpsilon = 1.0e-12;
constexpr double kQuaternionNormTolerance = 1.0e-3;

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & position = pose.position;
  const auto & orientation = pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
    !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
    !std::isfinite(orientation.w))
  {
    return false;
  }
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return std::isfinite(norm_squared) && norm_squared > kQuaternionNormEpsilon &&
         std::abs(norm_squared - 1.0) <= kQuaternionNormTolerance;
}

bool finiteTwist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

template<std::size_t Size>
bool finiteArray(const std::array<double, Size> & values)
{
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::Vector3 rotateVector(
  const geometry_msgs::msg::Vector3 & source,
  const tf2::Quaternion & body_from_world)
{
  const tf2::Vector3 source_vector(source.x, source.y, source.z);
  const tf2::Vector3 result = tf2::quatRotate(body_from_world, source_vector);
  geometry_msgs::msg::Vector3 output;
  output.x = result.x();
  output.y = result.y();
  output.z = result.z();
  return output;
}

std::array<double, 36> rotateCovariance(
  const std::array<double, 36> & source,
  const tf2::Matrix3x3 & body_from_world)
{
  double transform[6][6]{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      transform[row][column] = body_from_world[row][column];
      transform[row + 3][column + 3] = body_from_world[row][column];
    }
  }

  std::array<double, 36> result{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      double value = 0.0;
      for (std::size_t source_row = 0; source_row < 6; ++source_row) {
        for (std::size_t source_column = 0; source_column < 6; ++source_column) {
          value += transform[row][source_row] * source[source_row * 6 + source_column] *
            transform[column][source_column];
        }
      }
      result[row * 6 + column] = value;
    }
  }
  return result;
}
}  // namespace

const char * bodyOdometryStatusMessage(BodyOdometryStatus status)
{
  switch (status) {
    case BodyOdometryStatus::kOk:
      return "ok";
    case BodyOdometryStatus::kInvalidConfiguration:
      return "yaw offset or frame configuration is invalid";
    case BodyOdometryStatus::kFrameMismatch:
      return "input odometry parent or child frame is invalid";
    case BodyOdometryStatus::kInvalidPose:
      return "input odometry pose or pose covariance is invalid";
    case BodyOdometryStatus::kInvalidTwist:
      return "input odometry twist or twist covariance is invalid";
  }
  return "unknown body odometry status";
}

BodyOdometryStatus makeBodyOdometry(
  const nav_msgs::msg::Odometry & source,
  double yaw_offset,
  const std::string & world_frame,
  const std::string & body_frame,
  nav_msgs::msg::Odometry & output)
{
  if (!std::isfinite(yaw_offset) || std::abs(yaw_offset) > kPi ||
    world_frame.empty() || body_frame.empty())
  {
    return BodyOdometryStatus::kInvalidConfiguration;
  }
  if (source.header.frame_id != world_frame ||
    (!source.child_frame_id.empty() && source.child_frame_id != "imu"))
  {
    return BodyOdometryStatus::kFrameMismatch;
  }
  if (!finitePose(source.pose.pose) || !finiteArray(source.pose.covariance)) {
    return BodyOdometryStatus::kInvalidPose;
  }
  if (!finiteTwist(source.twist.twist) || !finiteArray(source.twist.covariance)) {
    return BodyOdometryStatus::kInvalidTwist;
  }

  const auto & source_orientation = source.pose.pose.orientation;
  tf2::Quaternion world_from_imu(
    source_orientation.x, source_orientation.y,
    source_orientation.z, source_orientation.w);
  world_from_imu.normalize();
  tf2::Quaternion imu_from_body;
  imu_from_body.setRPY(0.0, 0.0, yaw_offset);
  tf2::Quaternion world_from_body = world_from_imu * imu_from_body;
  world_from_body.normalize();

  nav_msgs::msg::Odometry candidate = source;
  candidate.child_frame_id = body_frame;
  candidate.pose.pose.orientation = tf2::toMsg(world_from_body);

  // Super-LIO leaves child_frame_id empty and stores state.v in world. If a
  // standards-compliant source identifies imu, its twist is already in imu.
  const tf2::Quaternion body_from_twist_frame = source.child_frame_id.empty() ?
    world_from_body.inverse() : imu_from_body.inverse();
  candidate.twist.twist.linear = rotateVector(
    source.twist.twist.linear, body_from_twist_frame);
  candidate.twist.twist.angular = rotateVector(
    source.twist.twist.angular, body_from_twist_frame);
  candidate.twist.covariance = rotateCovariance(
    source.twist.covariance, tf2::Matrix3x3(body_from_twist_frame));

  output = std::move(candidate);
  return BodyOdometryStatus::kOk;
}

}  // namespace utree_dog_navigation
