#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "utree_dog_navigation/body_odometry.hpp"

namespace utree_dog_navigation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

nav_msgs::msg::Odometry validOdometry(double yaw = 0.0)
{
  nav_msgs::msg::Odometry message;
  message.header.frame_id = "world";
  message.pose.pose.position.x = 1.0;
  message.pose.pose.position.y = 2.0;
  message.pose.pose.position.z = 3.0;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  message.pose.pose.orientation = tf2::toMsg(orientation);
  return message;
}
}  // namespace

TEST(BodyOdometry, AppliesClockwiseYawOffsetWithoutChangingPosition)
{
  auto source = validOdometry(0.25);
  source.header.stamp.sec = 123;
  source.header.stamp.nanosec = 456000000U;
  source.pose.covariance[0] = 0.25;
  source.pose.covariance[8] = 0.5;
  nav_msgs::msg::Odometry output;

  ASSERT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kOk);
  EXPECT_EQ(output.header.frame_id, "world");
  EXPECT_EQ(output.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.z, 3.0);
  EXPECT_EQ(output.header.stamp, source.header.stamp);
  EXPECT_EQ(output.pose.covariance, source.pose.covariance);
  EXPECT_NEAR(tf2::getYaw(output.pose.pose.orientation), 0.25 - 0.5 * kPi, 1.0e-12);
}

TEST(BodyOdometry, RightMultipliesOffsetForTiltedInput)
{
  auto source = validOdometry();
  tf2::Quaternion world_from_imu;
  world_from_imu.setRPY(0.2, -0.1, 0.4);
  source.pose.pose.orientation = tf2::toMsg(world_from_imu);
  tf2::Quaternion imu_from_body;
  imu_from_body.setRPY(0.0, 0.0, -0.5 * kPi);
  tf2::Quaternion expected = world_from_imu * imu_from_body;
  expected.normalize();

  nav_msgs::msg::Odometry output;
  ASSERT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kOk);
  const auto & actual_message = output.pose.pose.orientation;
  tf2::Quaternion actual(
    actual_message.x, actual_message.y, actual_message.z, actual_message.w);
  actual.normalize();
  EXPECT_NEAR(std::abs(actual.dot(expected)), 1.0, 1.0e-12);
}

TEST(BodyOdometry, RotatesWorldVelocityAndCovarianceIntoBodyFrame)
{
  auto source = validOdometry();
  source.twist.twist.linear.x = 1.0;
  source.twist.covariance[0] = 4.0;

  nav_msgs::msg::Odometry output;
  ASSERT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kOk);
  EXPECT_NEAR(output.twist.twist.linear.x, 0.0, 1.0e-12);
  EXPECT_NEAR(output.twist.twist.linear.y, 1.0, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[0], 0.0, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[7], 4.0, 1.0e-12);
}

TEST(BodyOdometry, RotatesStandardImuFrameTwistIntoBodyFrame)
{
  auto source = validOdometry(0.4);
  source.child_frame_id = "imu";
  source.twist.twist.linear.x = 1.0;
  source.twist.twist.angular.x = 2.0;
  source.twist.covariance[0] = 4.0;
  source.twist.covariance[3] = 1.5;
  source.twist.covariance[18] = 1.5;
  source.twist.covariance[21] = 9.0;

  nav_msgs::msg::Odometry output;
  ASSERT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kOk);
  EXPECT_NEAR(output.twist.twist.linear.x, 0.0, 1.0e-12);
  EXPECT_NEAR(output.twist.twist.linear.y, 1.0, 1.0e-12);
  EXPECT_NEAR(output.twist.twist.angular.x, 0.0, 1.0e-12);
  EXPECT_NEAR(output.twist.twist.angular.y, 2.0, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[7], 4.0, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[10], 1.5, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[25], 1.5, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[28], 9.0, 1.0e-12);
}

TEST(BodyOdometry, RejectsFrameMismatchAndInvalidQuaternion)
{
  auto source = validOdometry();
  source.header.frame_id = "map";
  nav_msgs::msg::Odometry output;
  EXPECT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kFrameMismatch);

  source = validOdometry();
  source.pose.pose.orientation.x = 0.0;
  source.pose.pose.orientation.y = 0.0;
  source.pose.pose.orientation.z = 0.0;
  source.pose.pose.orientation.w = 0.0;
  EXPECT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kInvalidPose);

  source = validOdometry();
  source.pose.pose.orientation.w = 2.0;
  EXPECT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kInvalidPose);

  source = validOdometry();
  source.child_frame_id = "base_link";
  EXPECT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kFrameMismatch);
}

TEST(BodyOdometry, RejectsInvalidConfigurationAndNonFiniteTwist)
{
  auto source = validOdometry();
  nav_msgs::msg::Odometry output;
  EXPECT_EQ(
    makeBodyOdometry(
      source, std::numeric_limits<double>::quiet_NaN(), "world", "base_link", output),
    BodyOdometryStatus::kInvalidConfiguration);

  source.twist.twist.linear.x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    makeBodyOdometry(source, -0.5 * kPi, "world", "base_link", output),
    BodyOdometryStatus::kInvalidTwist);
}

}  // namespace utree_dog_navigation
