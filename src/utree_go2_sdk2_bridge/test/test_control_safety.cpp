#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "utree_go2_sdk2_bridge/control_safety.hpp"

namespace utree_go2_sdk2_bridge
{
namespace
{
ControlParameters validParameters()
{
  return ControlParameters{
    20.0, 1.0, 0.5, 0.2, 0.6, 0.15, 0.2, 1.0, 1.5, 0.6, 0.35, 0.8};
}

geometry_msgs::msg::Pose validPose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = -2.0;
  pose.position.z = 0.3;
  pose.orientation.z = std::sin(0.25);
  pose.orientation.w = std::cos(0.25);
  return pose;
}
}  // namespace

TEST(ControlParameters, AcceptsShippedDefaults)
{
  EXPECT_TRUE(validateControlParameters(validParameters()).empty());
}

TEST(ControlParameters, RejectsNonFiniteAndOutOfRangeValues)
{
  auto parameters = validParameters();
  parameters.command_rate = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.command_rate = std::numeric_limits<double>::denorm_min();
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.command_rate = 201.0;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.path_timeout = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.goal_yaw_tolerance = 4.0;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.max_vy = -0.1;
  EXPECT_FALSE(validateControlParameters(parameters).empty());
}

TEST(PoseValidation, RejectsNonFinitePosition)
{
  auto pose = validPose();
  pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isFinitePose(pose));
}

TEST(PoseValidation, RejectsInvalidQuaternion)
{
  auto pose = validPose();
  pose.orientation = geometry_msgs::msg::Quaternion{};
  pose.orientation.w = 0.0;
  EXPECT_FALSE(isFinitePose(pose));
  EXPECT_FALSE(quaternionYaw(pose.orientation).has_value());

  pose = validPose();
  pose.orientation.w = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(isFinitePose(pose));

  pose = validPose();
  pose.orientation.w *= 2.0;
  EXPECT_FALSE(isFinitePose(pose));
}

TEST(PoseValidation, ComputesYawFromValidQuaternion)
{
  const auto yaw = quaternionYaw(validPose().orientation);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(*yaw, 0.5, 1.0e-12);
}

TEST(CommandValidation, ClampsFiniteCommands)
{
  const auto command = makeBoundedCommand(2.0, -2.0, 3.0, 0.6, 0.35, 0.8);
  ASSERT_TRUE(command.has_value());
  EXPECT_FLOAT_EQ(command->vx, 0.6F);
  EXPECT_FLOAT_EQ(command->vy, -0.35F);
  EXPECT_FLOAT_EQ(command->yaw_rate, 0.8F);
}

TEST(CommandValidation, RejectsNonFiniteRawValuesAndInvalidBounds)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(makeBoundedCommand(nan, 0.0, 0.0, 0.6, 0.35, 0.8).has_value());
  EXPECT_FALSE(makeBoundedCommand(0.0, 0.0, 0.0, -0.6, 0.35, 0.8).has_value());
  EXPECT_FALSE(makeBoundedCommand(
      0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(), 0.35, 0.8)
    .has_value());
}

}  // namespace utree_go2_sdk2_bridge
