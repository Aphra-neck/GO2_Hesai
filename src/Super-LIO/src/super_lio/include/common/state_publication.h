#ifndef LI2SUP_STATE_PUBLICATION_H_
#define LI2SUP_STATE_PUBLICATION_H_

#include <cmath>
#include <cstdint>
#include <limits>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "common/ds.h"

namespace LI2Sup {

// A field-equivalent fixture has determinant error around 0.0035 and
// orthogonality error around 0.0040. These bounds retain that estimator output
// while rejecting divergence.
inline constexpr BASIC::scalar kMaxPublishedQuaternionNormSquaredError = 1.0e-2F;
inline constexpr BASIC::scalar kMaxPublishedRotationDeterminantError = 2.0e-2F;
inline constexpr BASIC::scalar kMaxPublishedRotationOrthogonalityError = 2.0e-2F;

inline bool rotationDriftWithinLimits(
  BASIC::scalar determinant, BASIC::scalar orthogonality_error) noexcept
{
  return
    std::isfinite(determinant) &&
    std::isfinite(orthogonality_error) &&
    determinant > 0.0F &&
    orthogonality_error >= 0.0F &&
    std::abs(determinant - 1.0F) <=
    kMaxPublishedRotationDeterminantError &&
    orthogonality_error <= kMaxPublishedRotationOrthogonalityError;
}

enum class StatePublicationStatus
{
  kOk,
  kInvalidTimestamp,
  kNonfinitePosition,
  kNonfiniteVelocity,
  kNonfiniteRotation,
  kRotationDriftTooLarge,
  kInvalidQuaternion,
};

inline const char * statePublicationStatusName(StatePublicationStatus status) noexcept
{
  switch (status) {
    case StatePublicationStatus::kOk:
      return "ok";
    case StatePublicationStatus::kInvalidTimestamp:
      return "invalid_timestamp";
    case StatePublicationStatus::kNonfinitePosition:
      return "nonfinite_position";
    case StatePublicationStatus::kNonfiniteVelocity:
      return "nonfinite_velocity";
    case StatePublicationStatus::kNonfiniteRotation:
      return "nonfinite_rotation";
    case StatePublicationStatus::kRotationDriftTooLarge:
      return "rotation_drift_too_large";
    case StatePublicationStatus::kInvalidQuaternion:
      return "invalid_quaternion";
  }
  return "unknown";
}

struct PreparedStatePublication
{
  StatePublicationStatus status = StatePublicationStatus::kInvalidTimestamp;
  builtin_interfaces::msg::Time stamp;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Vector3 linear_velocity;
  BASIC::M3 rotation = BASIC::M3::Identity();
  BASIC::scalar raw_quaternion_norm_squared =
    std::numeric_limits<BASIC::scalar>::quiet_NaN();
  BASIC::scalar rotation_determinant =
    std::numeric_limits<BASIC::scalar>::quiet_NaN();
  BASIC::scalar rotation_orthogonality_error =
    std::numeric_limits<BASIC::scalar>::quiet_NaN();

  bool valid() const noexcept
  {
    return status == StatePublicationStatus::kOk;
  }
};

inline PreparedStatePublication prepareStatePublication(const NavState & state) noexcept
{
  PreparedStatePublication prepared;

  if (!std::isfinite(state.timestamp)) {
    return prepared;
  }
  const double whole_seconds = std::floor(state.timestamp);
  if (
    whole_seconds < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
    whole_seconds > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
  {
    return prepared;
  }
  // Match the point-cloud publisher exactly so cloud/odometry stamps still pair.
  prepared.stamp.sec = static_cast<std::int32_t>(whole_seconds);
  prepared.stamp.nanosec = static_cast<std::uint32_t>(
    (state.timestamp - whole_seconds) * 1.0e9);

  if (!state.p.allFinite()) {
    prepared.status = StatePublicationStatus::kNonfinitePosition;
    return prepared;
  }
  if (!state.v.allFinite()) {
    prepared.status = StatePublicationStatus::kNonfiniteVelocity;
    return prepared;
  }

  const BASIC::M3 raw_rotation = state.R.matrix();
  if (!raw_rotation.allFinite()) {
    prepared.status = StatePublicationStatus::kNonfiniteRotation;
    return prepared;
  }

  prepared.rotation_determinant = raw_rotation.determinant();
  prepared.rotation_orthogonality_error =
    (raw_rotation.transpose() * raw_rotation - BASIC::M3::Identity()).norm();
  if (!rotationDriftWithinLimits(
      prepared.rotation_determinant,
      prepared.rotation_orthogonality_error))
  {
    prepared.status = StatePublicationStatus::kRotationDriftTooLarge;
    return prepared;
  }

  BASIC::Quat orientation(raw_rotation);
  prepared.raw_quaternion_norm_squared = orientation.squaredNorm();
  if (
    !orientation.coeffs().allFinite() ||
    !std::isfinite(prepared.raw_quaternion_norm_squared) ||
    prepared.raw_quaternion_norm_squared <=
    std::numeric_limits<BASIC::scalar>::epsilon() ||
    std::abs(prepared.raw_quaternion_norm_squared - 1.0F) >
    kMaxPublishedQuaternionNormSquaredError)
  {
    prepared.status = StatePublicationStatus::kInvalidQuaternion;
    return prepared;
  }

  orientation.normalize();
  if (!orientation.coeffs().allFinite()) {
    prepared.status = StatePublicationStatus::kInvalidQuaternion;
    return prepared;
  }

  prepared.pose.position.x = static_cast<double>(state.p[0]);
  prepared.pose.position.y = static_cast<double>(state.p[1]);
  prepared.pose.position.z = static_cast<double>(state.p[2]);
  prepared.pose.orientation.x = static_cast<double>(orientation.x());
  prepared.pose.orientation.y = static_cast<double>(orientation.y());
  prepared.pose.orientation.z = static_cast<double>(orientation.z());
  prepared.pose.orientation.w = static_cast<double>(orientation.w());
  prepared.linear_velocity.x = static_cast<double>(state.v[0]);
  prepared.linear_velocity.y = static_cast<double>(state.v[1]);
  prepared.linear_velocity.z = static_cast<double>(state.v[2]);
  prepared.rotation = orientation.toRotationMatrix();
  prepared.status = StatePublicationStatus::kOk;
  return prepared;
}

inline void populateWorldHeader(
  const PreparedStatePublication & prepared,
  std_msgs::msg::Header & header)
{
  header = std_msgs::msg::Header{};
  header.stamp = prepared.stamp;
  header.frame_id = "world";
}

inline void populateWorldPointCloudHeader(
  const PreparedStatePublication & prepared,
  sensor_msgs::msg::PointCloud2 & cloud)
{
  populateWorldHeader(prepared, cloud.header);
}

inline void populateStateMessages(
  const PreparedStatePublication & prepared,
  nav_msgs::msg::Odometry & odometry,
  geometry_msgs::msg::TransformStamped & transform)
{
  odometry = nav_msgs::msg::Odometry{};
  populateWorldHeader(prepared, odometry.header);
  odometry.pose.pose = prepared.pose;
  odometry.twist.twist.linear = prepared.linear_velocity;
  // The velocity is world-frame, so do not label it as an imu-frame twist.

  transform = geometry_msgs::msg::TransformStamped{};
  populateWorldHeader(prepared, transform.header);
  transform.child_frame_id = "imu";
  transform.transform.translation.x = prepared.pose.position.x;
  transform.transform.translation.y = prepared.pose.position.y;
  transform.transform.translation.z = prepared.pose.position.z;
  transform.transform.rotation = prepared.pose.orientation;
}

}  // namespace LI2Sup

#endif  // LI2SUP_STATE_PUBLICATION_H_
