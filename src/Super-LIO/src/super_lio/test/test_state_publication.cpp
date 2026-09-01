#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "common/state_publication.h"

namespace LI2Sup {
namespace {

NavState validState()
{
  NavState state;
  state.timestamp = 1234.25;
  state.p << 1.0F, -2.0F, 0.5F;
  state.v << 0.1F, -0.2F, 0.3F;
  return state;
}

TEST(StatePublicationTest, NormalizesBoundedSuperLioRotationDrift)
{
  NavState state = validState();
  BASIC::M3 drifted_rotation;
  drifted_rotation <<
    -0.3008896258F, -0.9542911456F, 0.0332201641F,
    0.9533738746F, -0.2982880827F, 0.0664244813F,
    -0.0534174607F, 0.0515981192F, 0.9983956860F;
  state.R = BASIC::SO3(drifted_rotation);

  const BASIC::Quat raw_orientation(drifted_rotation);
  ASSERT_GT(
    std::abs(raw_orientation.squaredNorm() - 1.0F),
    1.0e-3F);
  ASSERT_NEAR(
    raw_orientation.squaredNorm(), 1.001402706F, 2.0e-6F);

  const PreparedStatePublication prepared = prepareStatePublication(state);

  ASSERT_EQ(prepared.status, StatePublicationStatus::kOk);
  EXPECT_FLOAT_EQ(
    prepared.raw_quaternion_norm_squared,
    raw_orientation.squaredNorm());
  nav_msgs::msg::Odometry odometry;
  geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::msg::PointCloud2 cloud;
  populateStateMessages(prepared, odometry, transform);
  populateWorldPointCloudHeader(prepared, cloud);

  const auto & odom_q = odometry.pose.pose.orientation;
  const auto & tf_q = transform.transform.rotation;
  const double published_norm_squared =
    odom_q.x * odom_q.x + odom_q.y * odom_q.y +
    odom_q.z * odom_q.z + odom_q.w * odom_q.w;
  EXPECT_NEAR(published_norm_squared, 1.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(odom_q.x, tf_q.x);
  EXPECT_DOUBLE_EQ(odom_q.y, tf_q.y);
  EXPECT_DOUBLE_EQ(odom_q.z, tf_q.z);
  EXPECT_DOUBLE_EQ(odom_q.w, tf_q.w);
  EXPECT_EQ(odometry.header.stamp, transform.header.stamp);
  EXPECT_EQ(odometry.header.stamp, cloud.header.stamp);
  EXPECT_EQ(odometry.header.frame_id, "world");
  EXPECT_EQ(transform.header.frame_id, "world");
  EXPECT_EQ(cloud.header.frame_id, "world");
  EXPECT_EQ(transform.child_frame_id, "imu");
  EXPECT_NEAR(prepared.rotation.determinant(), 1.0F, 1.0e-5F);
  EXPECT_NEAR(
    (prepared.rotation.transpose() * prepared.rotation -
    BASIC::M3::Identity()).norm(),
    0.0F, 1.0e-5F);
}

TEST(StatePublicationTest, UsesOneExactStampForOdomTfAndWorldCloud)
{
  NavState state = validState();
  state.timestamp = 1234.1234567896;
  const PreparedStatePublication prepared = prepareStatePublication(state);
  ASSERT_TRUE(prepared.valid());

  nav_msgs::msg::Odometry odometry;
  geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::msg::PointCloud2 cloud;
  populateStateMessages(prepared, odometry, transform);
  populateWorldPointCloudHeader(prepared, cloud);

  EXPECT_EQ(odometry.header.stamp, transform.header.stamp);
  EXPECT_EQ(odometry.header.stamp, cloud.header.stamp);
  EXPECT_EQ(odometry.header.frame_id, "world");
  EXPECT_EQ(transform.header.frame_id, "world");
  EXPECT_EQ(cloud.header.frame_id, "world");
  EXPECT_EQ(transform.child_frame_id, "imu");
}

TEST(StatePublicationTest, EnforcesRotationMetricBoundaries)
{
  const BASIC::scalar infinity =
    std::numeric_limits<BASIC::scalar>::infinity();
  const BASIC::scalar upper_determinant =
    1.0F + kMaxPublishedRotationDeterminantError;
  const BASIC::scalar lower_determinant =
    1.0F - kMaxPublishedRotationDeterminantError;
  const BASIC::scalar determinant_inside_high =
    std::nextafter(upper_determinant, 1.0F);
  const BASIC::scalar determinant_outside_high =
    std::nextafter(upper_determinant, infinity);
  const BASIC::scalar determinant_inside_low =
    std::nextafter(lower_determinant, 1.0F);
  const BASIC::scalar determinant_outside_low =
    std::nextafter(lower_determinant, 0.0F);
  const BASIC::scalar orthogonality_inside = std::nextafter(
    kMaxPublishedRotationOrthogonalityError, 0.0F);
  const BASIC::scalar orthogonality_outside = std::nextafter(
    kMaxPublishedRotationOrthogonalityError, infinity);

  EXPECT_TRUE(rotationDriftWithinLimits(
    determinant_inside_high, orthogonality_inside));
  EXPECT_FALSE(rotationDriftWithinLimits(
    determinant_outside_high, orthogonality_inside));
  EXPECT_TRUE(rotationDriftWithinLimits(
    determinant_inside_low, orthogonality_inside));
  EXPECT_FALSE(rotationDriftWithinLimits(
    determinant_outside_low, orthogonality_inside));
  EXPECT_TRUE(rotationDriftWithinLimits(1.0F, orthogonality_inside));
  EXPECT_FALSE(rotationDriftWithinLimits(1.0F, orthogonality_outside));
}

TEST(StatePublicationTest, PreservesFiniteStateFields)
{
  const NavState state = validState();

  const PreparedStatePublication prepared = prepareStatePublication(state);

  ASSERT_TRUE(prepared.valid());
  EXPECT_EQ(prepared.stamp.sec, 1234);
  EXPECT_EQ(prepared.stamp.nanosec, 250000000U);
  EXPECT_DOUBLE_EQ(prepared.pose.position.x, static_cast<double>(state.p[0]));
  EXPECT_DOUBLE_EQ(prepared.pose.position.y, static_cast<double>(state.p[1]));
  EXPECT_DOUBLE_EQ(prepared.pose.position.z, static_cast<double>(state.p[2]));
  EXPECT_DOUBLE_EQ(
    prepared.linear_velocity.x, static_cast<double>(state.v[0]));
  EXPECT_DOUBLE_EQ(
    prepared.linear_velocity.y, static_cast<double>(state.v[1]));
  EXPECT_DOUBLE_EQ(
    prepared.linear_velocity.z, static_cast<double>(state.v[2]));
}

TEST(StatePublicationTest, RejectsNonfiniteStateFields)
{
  NavState state = validState();
  state.timestamp = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    prepareStatePublication(state).status,
    StatePublicationStatus::kInvalidTimestamp);

  state = validState();
  state.p[1] = std::numeric_limits<BASIC::scalar>::infinity();
  EXPECT_EQ(
    prepareStatePublication(state).status,
    StatePublicationStatus::kNonfinitePosition);

  state = validState();
  state.v[2] = std::numeric_limits<BASIC::scalar>::quiet_NaN();
  EXPECT_EQ(
    prepareStatePublication(state).status,
    StatePublicationStatus::kNonfiniteVelocity);

  state = validState();
  BASIC::M3 nonfinite_rotation = BASIC::M3::Identity();
  nonfinite_rotation(0, 0) =
    std::numeric_limits<BASIC::scalar>::quiet_NaN();
  state.R = BASIC::SO3(nonfinite_rotation);
  EXPECT_EQ(
    prepareStatePublication(state).status,
    StatePublicationStatus::kNonfiniteRotation);
}

TEST(StatePublicationTest, RejectsDegenerateOrDivergentRotations)
{
  NavState state = validState();
  state.R = BASIC::SO3(BASIC::M3::Zero());
  EXPECT_EQ(
    prepareStatePublication(state).status,
    StatePublicationStatus::kRotationDriftTooLarge);

  state = validState();
  BASIC::M3 reflection = BASIC::M3::Identity();
  reflection(2, 2) = -1.0F;
  state.R = BASIC::SO3(reflection);
  EXPECT_EQ(
    prepareStatePublication(state).status,
    StatePublicationStatus::kRotationDriftTooLarge);

  state = validState();
  state.R = BASIC::SO3(2.0F * BASIC::M3::Identity());
  EXPECT_EQ(
    prepareStatePublication(state).status,
    StatePublicationStatus::kRotationDriftTooLarge);

  state = validState();
  BASIC::M3 shear = BASIC::M3::Identity();
  shear(0, 1) = 0.1F;
  state.R = BASIC::SO3(shear);
  EXPECT_EQ(
    prepareStatePublication(state).status,
    StatePublicationStatus::kRotationDriftTooLarge);
}

}  // namespace
}  // namespace LI2Sup
