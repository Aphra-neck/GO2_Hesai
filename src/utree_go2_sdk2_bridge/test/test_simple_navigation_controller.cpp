#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "utree_go2_sdk2_bridge/simple_navigation_controller.hpp"

namespace utree_go2_sdk2_bridge
{

namespace
{

SimpleNavigationConfig testConfig()
{
  SimpleNavigationConfig config;
  // Keep the pure-controller test on the same defaults used by the direct
  // bridge YAML.  A separate tuning profile here can hide corner behavior
  // that will be seen on the Jetson.
  config.position_tolerance = 0.15;
  config.yaw_tolerance = 0.12;
  config.align_tolerance = 0.08;
  config.waypoint_cross_track_tolerance = 0.30;
  config.linear_gain = 1.0;
  config.lateral_gain = 1.0;
  config.yaw_gain = 1.5;
  config.max_vx = 0.6;
  config.max_vy = 0.35;
  config.max_yaw_rate = 0.8;
  return config;
}

double wrap(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  return std::remainder(angle, 2.0 * kPi);
}

SimpleNavigationPose integrate(
  const SimpleNavigationPose & pose, const SimpleNavigationCommand & command, double dt)
{
  const double next_yaw = wrap(pose.yaw + command.yaw_rate * dt);
  return {
    pose.x + (std::cos(pose.yaw) * command.vx - std::sin(pose.yaw) * command.vy) * dt,
    pose.y + (std::sin(pose.yaw) * command.vx + std::cos(pose.yaw) * command.vy) * dt,
    next_yaw};
}

}

TEST(SimpleNavigationController, LongRightAngleRouteConvergesWithoutZeroDeadlock)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({4.0, 3.0, 0.0});

  SimpleNavigationPose pose{};
  constexpr double dt = 0.05;
  bool reached = false;
  for (int step = 0; step < 600; ++step) {
    const auto command = controller.update(pose);
    ASSERT_TRUE(command.valid) << "step=" << step;
    pose = integrate(pose, command, dt);
    if (command.goal_reached) {
      reached = true;
      break;
    }
  }
  EXPECT_TRUE(reached);
  EXPECT_NEAR(pose.x, 4.0, 0.20);
  EXPECT_NEAR(pose.y, 3.0, 0.20);
}

TEST(SimpleNavigationController, OvershotWaypointAdvancesToNextSegment)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({2.0, 1.0, 0.0});

  const auto first = controller.update({0.0, 0.0, 0.0});
  ASSERT_TRUE(first.valid);
  EXPECT_GT(first.vx, 0.0);

  const auto after_overshoot = controller.update({2.25, 0.0, 0.0});
  ASSERT_TRUE(after_overshoot.valid);
  EXPECT_GE(after_overshoot.waypoints_reached, 1U);
  // Passing a corner does not make the next leg aligned.  The first command
  // after the pass must rotate in place; translation resumes on the next
  // odometry update once the new segment heading is reached.
  EXPECT_EQ(after_overshoot.phase, SimpleNavigationPhase::kAlignSegment);
  EXPECT_NEAR(after_overshoot.vx, 0.0, 1.0e-12);
  EXPECT_NEAR(after_overshoot.vy, 0.0, 1.0e-12);
  EXPECT_GT(after_overshoot.yaw_rate, 0.0);

  const auto aligned = controller.update({2.25, 0.0, 1.5707963267948966});
  ASSERT_TRUE(aligned.valid);
  EXPECT_EQ(aligned.phase, SimpleNavigationPhase::kTranslateSegment);
  EXPECT_GT(aligned.vx, 0.0);
  EXPECT_GT(aligned.vy, 0.0);
}

TEST(SimpleNavigationController, CornerEmitsNextTranslationInSameTick)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({2.0, 1.0, 0.0});

  const auto first = controller.update({0.0, 0.0, 0.0});
  ASSERT_TRUE(first.valid);
  ASSERT_GT(first.vx, 0.0);

  // The corner is already reached at the beginning of this update.  The
  // controller must advance and emit the next leg immediately, rather than
  // returning an invalid/zero command for one 20 Hz cycle.
  const auto corner = controller.update({2.0, 0.0, 0.0});
  EXPECT_TRUE(corner.valid);
  EXPECT_TRUE(corner.segment_aligned);
  EXPECT_EQ(corner.waypoints_reached, 1U);
  EXPECT_EQ(corner.phase, SimpleNavigationPhase::kTranslateSegment);
  EXPECT_GT(corner.vy, 0.0);
  EXPECT_LE(std::abs(corner.vx), 1.0e-12);
}

TEST(SimpleNavigationController, FinalTargetOvershootCommandsBackInsteadOfCompleting)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({2.0, 0.0, 0.0});

  const auto initial = controller.update({0.0, 0.0, 0.0});
  ASSERT_TRUE(initial.valid);

  const auto command = controller.update({2.40, 0.0, 0.0});
  ASSERT_TRUE(command.valid);
  EXPECT_FALSE(command.goal_reached);
  EXPECT_LT(command.vx, 0.0);
  EXPECT_TRUE(controller.hasGoal());
  EXPECT_EQ(controller.routeIndex(), 0U);
}

TEST(SimpleNavigationController, NonzeroInitialYawUsesBodyForwardForTranslation)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({0.0, 2.0, 0.0});

  constexpr double kHalfPi = 1.5707963267948966;
  const auto command = controller.update({0.0, 0.0, kHalfPi});
  ASSERT_TRUE(command.valid);
  EXPECT_TRUE(command.segment_aligned);
  EXPECT_GT(command.vx, 0.0);
  EXPECT_NEAR(command.vy, 0.0, 1.0e-12);
  EXPECT_NEAR(command.yaw_rate, 0.0, 1.0e-12);
}

TEST(SimpleNavigationController, HeadingWraparoundDoesNotRotateTheLongWay)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({-1.0, 0.0, 0.0});

  constexpr double kNearlyPi = 3.13;
  const auto command = controller.update({0.0, 0.0, kNearlyPi});
  ASSERT_TRUE(command.valid);
  EXPECT_TRUE(command.segment_aligned);
  EXPECT_GT(command.vx, 0.0);
  EXPECT_LT(std::abs(command.yaw_rate), 0.1);
}

TEST(SimpleNavigationController, NewGoalRebuildsFromCurrentPoseAfterCompletion)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({0.0, 0.0, 0.0});

  const auto first = controller.update({0.0, 0.0, 0.0});
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(first.goal_reached);
  EXPECT_FALSE(controller.hasGoal());

  controller.setGoal({1.0, 0.0, 0.0});
  const auto second = controller.update({0.0, 0.0, 0.0});
  ASSERT_TRUE(second.valid);
  EXPECT_TRUE(second.segment_aligned);
  EXPECT_GT(second.vx, 0.0);
}

TEST(SimpleNavigationController, LongRouteHasNoInvalidTickAtCorner)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({4.0, 3.0, 0.0});

  SimpleNavigationPose pose{};
  constexpr double dt = 0.05;
  int last_valid_step = -1;
  int max_valid_step_gap = 0;
  bool reached = false;
  for (int step = 0; step < 600; ++step) {
    const auto command = controller.update(pose);
    ASSERT_TRUE(command.valid) << "step=" << step;
    if (last_valid_step >= 0) {
      max_valid_step_gap = std::max(max_valid_step_gap, step - last_valid_step);
    }
    last_valid_step = step;
    pose = integrate(pose, command, dt);
    if (command.goal_reached) {
      reached = true;
      break;
    }
  }
  EXPECT_TRUE(reached);
  EXPECT_EQ(max_valid_step_gap, 1);
}

TEST(SimpleNavigationController, GoalCompletionKeepsZeroSpeedCommandValid)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({0.0, 0.0, 0.0});

  const auto command = controller.update({0.0, 0.0, 0.0});
  EXPECT_TRUE(command.valid);
  EXPECT_TRUE(command.goal_reached);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_DOUBLE_EQ(command.yaw_rate, 0.0);
  EXPECT_FALSE(controller.hasGoal());
}

TEST(SimpleNavigationController, HeadingAlignmentFallsThroughToTranslation)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({1.0, 0.0, 0.0});

  const auto command = controller.update({0.0, 0.0, 0.0});
  EXPECT_TRUE(command.valid);
  EXPECT_TRUE(command.segment_aligned);
  EXPECT_GT(command.vx, 0.0);
}

TEST(SimpleNavigationController, InvalidPoseDoesNotProduceAnSDKCommand)
{
  SimpleNavigationController controller;
  controller.setConfig(testConfig());
  controller.setGoal({1.0, 0.0, 0.0});

  SimpleNavigationPose pose{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  const auto command = controller.update(pose);
  EXPECT_FALSE(command.valid);
  EXPECT_FALSE(command.goal_reached);
  EXPECT_TRUE(controller.hasGoal());
}

TEST(SimpleNavigationController, InvalidConfigDoesNotProduceAnSDKCommand)
{
  SimpleNavigationController controller;
  auto config = testConfig();
  config.max_vx = std::numeric_limits<double>::quiet_NaN();
  controller.setConfig(config);
  controller.setGoal({1.0, 0.0, 0.0});

  const auto command = controller.update({0.0, 0.0, 0.0});
  EXPECT_FALSE(command.valid);
  EXPECT_FALSE(command.goal_reached);
}

}  // namespace utree_go2_sdk2_bridge
