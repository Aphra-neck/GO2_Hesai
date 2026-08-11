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
    20.0, 1.0, 0.5, 0.2, 0.6, 0.15, 0.2,
    0.7853981633974483, 0.2617993877991494,
    1.0, 1.5, 0.1, 0.05, 0.2};
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

geometry_msgs::msg::PoseStamped pathPose(
  double x, double y, double yaw = 0.0, std::int64_t goal_generation = 1000000001LL)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp.sec = static_cast<std::int32_t>(goal_generation / 1000000000LL);
  pose.header.stamp.nanosec = static_cast<std::uint32_t>(goal_generation % 1000000000LL);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(0.5 * yaw);
  pose.pose.orientation.w = std::cos(0.5 * yaw);
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
  parameters.heading_alignment_exit_angle = parameters.heading_alignment_enter_angle;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.max_vy = -0.1;
  EXPECT_FALSE(validateControlParameters(parameters).empty());
}

TEST(ControlParameters, RejectsValuesAboveTheValidatedFlatStageCaps)
{
  auto parameters = validParameters();
  parameters.max_vx = 0.100001;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.max_vy = 0.050001;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.max_yaw_rate = 0.200001;
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

TEST(HeadingAlignment, UsesHysteresisForRotateInPlaceGate)
{
  constexpr double enter = 0.7853981633974483;
  constexpr double exit = 0.2617993877991494;

  const auto entered = updateHeadingAlignmentGate(false, 1.2, enter, exit);
  ASSERT_TRUE(entered.has_value());
  EXPECT_TRUE(*entered);

  const auto held = updateHeadingAlignmentGate(*entered, 0.5, enter, exit);
  ASSERT_TRUE(held.has_value());
  EXPECT_TRUE(*held);

  const auto exited = updateHeadingAlignmentGate(*held, 0.2, enter, exit);
  ASSERT_TRUE(exited.has_value());
  EXPECT_FALSE(*exited);

  const auto stayed_open = updateHeadingAlignmentGate(*exited, 0.5, enter, exit);
  ASSERT_TRUE(stayed_open.has_value());
  EXPECT_FALSE(*stayed_open);
}

TEST(HeadingAlignment, RejectsInvalidInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(updateHeadingAlignmentGate(false, nan, 0.8, 0.2).has_value());
  EXPECT_FALSE(updateHeadingAlignmentGate(false, 1.0, 0.2, 0.2).has_value());
}

TEST(HeadingAlignment, UsesFinalGoalYawInsideTheGoalPositionTolerance)
{
  const auto approaching = selectAlignmentYawError(0.1, 1.2, 0.16, 0.15);
  ASSERT_TRUE(approaching.has_value());
  EXPECT_DOUBLE_EQ(*approaching, 0.1);

  const auto at_goal_position = selectAlignmentYawError(0.0, 1.2, 0.10, 0.15);
  ASSERT_TRUE(at_goal_position.has_value());
  EXPECT_DOUBLE_EQ(*at_goal_position, 1.2);
}

TEST(HeadingAlignment, ExplicitOneBinTurnStillRequiresRotateOnlyExecution)
{
  constexpr double one_yaw_bin = 0.39269908169872414;
  constexpr double exit_angle = 0.2617993877991494;

  const auto explicit_turn = requireRotateInPlace(
    false, true, one_yaw_bin, exit_angle, 1.0, 0.15);
  ASSERT_TRUE(explicit_turn.has_value());
  EXPECT_TRUE(*explicit_turn);

  const auto ordinary_tracking = requireRotateInPlace(
    false, false, one_yaw_bin, exit_angle, 1.0, 0.15);
  ASSERT_TRUE(ordinary_tracking.has_value());
  EXPECT_FALSE(*ordinary_tracking);

  const auto aligned = requireRotateInPlace(
    false, true, 0.2, exit_angle, 1.0, 0.15);
  ASSERT_TRUE(aligned.has_value());
  EXPECT_FALSE(*aligned);
}

TEST(HeadingAlignment, SuppressesTranslationButPreservesYawCommand)
{
  const auto command = makeHeadingAwareCommand(
    -0.6, 0.35, 0.7, true, 0.6, 0.35, 0.8);

  ASSERT_TRUE(command.has_value());
  EXPECT_FLOAT_EQ(command->vx, 0.0F);
  EXPECT_FLOAT_EQ(command->vy, 0.0F);
  EXPECT_FLOAT_EQ(command->yaw_rate, 0.7F);
}

TEST(PathProgress, RejectsAnAmbiguousPositionBetweenTheBranchesOfAUTurn)
{
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0),
    pathPose(0.4, 0.0),
    pathPose(0.8, 0.0),
    pathPose(0.8, 0.2, 1.5707963267948966),
    pathPose(0.4, 0.2, 3.1415926535897932),
    pathPose(0.0, 0.2, 3.1415926535897932)};
  PathProgressTracker tracker;

  EXPECT_FALSE(
    tracker.update(poses, 0.2, 0.11, 0.0, 0.3, 0.2617993877991494).has_value());
}

TEST(PathProgress, TracksABoundedDeviationWithoutJumpingPastTheCurrentSegment)
{
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0),
    pathPose(0.4, 0.0),
    pathPose(0.8, 0.0),
    pathPose(0.8, 0.2, 1.5707963267948966),
    pathPose(0.4, 0.2, 3.1415926535897932),
    pathPose(0.0, 0.2, 3.1415926535897932)};
  PathProgressTracker tracker;

  const auto target = tracker.update(poses, 0.2, 0.04, 0.0, 0.3, 0.2617993877991494);

  ASSERT_TRUE(target.has_value());
  EXPECT_EQ(target->progress_pose, 0U);
  EXPECT_NEAR(target->target_y, 0.0, 1.0e-12);
  EXPECT_GT(target->target_x, 0.2);
}

TEST(PathProgress, NeverMovesTheCursorBackwardOnTheSamePath)
{
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0), pathPose(0.2, 0.0), pathPose(0.4, 0.0)};
  PathProgressTracker tracker;

  const auto first = tracker.update(poses, 0.10, 0.0, 0.0, 0.2, 0.1);
  ASSERT_TRUE(first.has_value());
  const auto slightly_backward = tracker.update(poses, 0.09, 0.0, 0.0, 0.2, 0.1);
  ASSERT_TRUE(slightly_backward.has_value());

  EXPECT_EQ(slightly_backward->progress_pose, first->progress_pose);
  EXPECT_DOUBLE_EQ(slightly_backward->progress_fraction, first->progress_fraction);
}

TEST(PathProgress, RequiresSignedIncomingProgressBeforeConsumingACornerTurn)
{
  constexpr double half_pi = 1.5707963267948966;
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(-0.2, 0.0),
    pathPose(0.0, 0.0),
    pathPose(0.0, 0.0, half_pi),
    pathPose(0.0, 0.2, half_pi)};
  PathProgressTracker tracker;

  const auto early = tracker.update(poses, -0.04, 0.0, 0.0, 0.3, 0.2617993877991494);
  ASSERT_TRUE(early.has_value());
  EXPECT_EQ(early->heading_pose, 0U);
  EXPECT_FALSE(early->explicit_rotation_waypoint);
  EXPECT_NEAR(early->target_x, 0.0, 1.0e-12);

  const auto crossed = tracker.update(poses, 0.01, 0.0, 0.0, 0.3, 0.2617993877991494);
  ASSERT_TRUE(crossed.has_value());
  EXPECT_EQ(crossed->heading_pose, 2U);
  EXPECT_TRUE(crossed->explicit_rotation_waypoint);
}

TEST(PathProgress, OvershotCornerRotatesThenTargetsForwardWithoutBackingUp)
{
  constexpr double half_pi = 1.5707963267948966;
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0),
    pathPose(0.2, 0.0),
    pathPose(0.4, 0.0),
    pathPose(0.4, 0.0, half_pi),
    pathPose(0.4, 0.2, half_pi),
    pathPose(0.4, 0.4, half_pi)};
  PathProgressTracker tracker;

  const auto rotate = tracker.update(poses, 0.42, 0.0, 0.0, 0.3, 0.2617993877991494);
  ASSERT_TRUE(rotate.has_value());
  EXPECT_TRUE(rotate->explicit_rotation_waypoint);

  const auto forward = tracker.update(poses, 0.42, 0.0, half_pi, 0.3, 0.2617993877991494);
  ASSERT_TRUE(forward.has_value());
  EXPECT_FALSE(forward->explicit_rotation_waypoint);
  EXPECT_FALSE(forward->reverse_motion);
  const double body_dx = forward->target_y;
  ASSERT_GT(body_dx, 0.0);
  const auto filtered = rejectUnexpectedReverseCommand(body_dx, forward->reverse_motion);
  ASSERT_TRUE(filtered.has_value());
  EXPECT_GT(*filtered, 0.0);
}

TEST(PathProgress, ConsumesMultiBinRotationInThePlannedOrder)
{
  constexpr double pi = 3.1415926535897932;
  constexpr double yaw_step = pi / 8.0;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (std::size_t index = 0U; index <= 8U; ++index) {
    poses.push_back(pathPose(0.0, 0.0, static_cast<double>(index) * yaw_step));
  }
  PathProgressTracker tracker;

  for (std::size_t index = 0U; index < 8U; ++index) {
    const auto target = tracker.update(
      poses, 0.0, 0.0, static_cast<double>(index) * yaw_step, 0.3, 0.1);
    ASSERT_TRUE(target.has_value());
    EXPECT_TRUE(target->explicit_rotation_waypoint);
    EXPECT_EQ(target->heading_pose, index + 1U);
  }
}

TEST(PathProgress, RejectsRotationWhenTheBodyIsNotAtTheWaypoint)
{
  constexpr double half_pi = 1.5707963267948966;
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0, 0.0),
    pathPose(0.0, 0.0, half_pi)};
  PathProgressTracker tracker;

  EXPECT_FALSE(
    tracker.update(poses, 0.20, 0.0, 0.0, 0.3, 0.1).has_value());
}

TEST(PathProgress, RejectsUnconfirmedProgressInsteadOfJumpingForward)
{
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0), pathPose(0.2, 0.0), pathPose(0.4, 0.0),
    pathPose(0.6, 0.0), pathPose(0.8, 0.0), pathPose(1.0, 0.0)};
  PathProgressTracker tracker;

  EXPECT_FALSE(
    tracker.update(poses, 1.0, 0.0, 0.0, 0.3, 0.2617993877991494).has_value());
}

TEST(PathProgress, RejectsASinglePosePathAwayFromTheRobot)
{
  PathProgressTracker tracker;
  const std::vector<geometry_msgs::msg::PoseStamped> poses{pathPose(1.0, 0.0)};

  EXPECT_FALSE(
    tracker.update(poses, 0.0, 0.0, 0.0, 0.2, 0.1).has_value());
}

TEST(PathProgress, RejectsLateralJumpAfterReachingTheFinalPose)
{
  PathProgressTracker tracker;
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0), pathPose(0.2, 0.0)};

  ASSERT_TRUE(tracker.update(poses, 0.2, 0.0, 0.0, 0.2, 0.1).has_value());
  EXPECT_FALSE(tracker.update(poses, 0.2, 0.06, 0.0, 0.2, 0.1).has_value());
}

TEST(CommandValidation, RejectsUnexpectedReverseButAllowsAnExplicitReverseSegment)
{
  EXPECT_FALSE(rejectUnexpectedReverseCommand(-0.02, false).has_value());

  const auto numerical_noise = rejectUnexpectedReverseCommand(-1.0e-5, false);
  ASSERT_TRUE(numerical_noise.has_value());
  EXPECT_DOUBLE_EQ(*numerical_noise, 0.0);

  const auto planned_reverse = rejectUnexpectedReverseCommand(-0.02, true);
  ASSERT_TRUE(planned_reverse.has_value());
  EXPECT_DOUBLE_EQ(*planned_reverse, -0.02);
}

TEST(PathProgress, MarksAPlannerReversePrimitiveExplicitly)
{
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0, 0.0), pathPose(-0.2, 0.0, 0.0)};
  PathProgressTracker tracker;

  const auto target = tracker.update(poses, 0.0, 0.0, 0.0, 0.2, 0.1);

  ASSERT_TRUE(target.has_value());
  EXPECT_TRUE(target->reverse_motion);
  const auto command = rejectUnexpectedReverseCommand(target->target_x, true);
  ASSERT_TRUE(command.has_value());
  EXPECT_LT(*command, 0.0);
}

TEST(CompletedGoal, ExtractsOneGoalGenerationFromEveryPathPose)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0, 0.0, 2000000003LL),
    pathPose(0.2, 0.0, 0.0, 2000000003LL)};

  const auto generation = pathGoalGeneration(poses);
  ASSERT_TRUE(generation.has_value());
  EXPECT_EQ(*generation, 2000000003LL);

  poses.back().header.stamp.nanosec = 4U;
  EXPECT_FALSE(pathGoalGeneration(poses).has_value());
  poses.front().header.stamp.sec = 0;
  poses.front().header.stamp.nanosec = 0U;
  poses.back().header.stamp.sec = 0;
  poses.back().header.stamp.nanosec = 0U;
  EXPECT_FALSE(pathGoalGeneration(poses).has_value());
  EXPECT_FALSE(pathGoalGeneration({}).has_value());
}

TEST(CompletedGoal, BlocksEveryReplanOfTheCompletedGoalGeneration)
{
  CompletedGoalLatch latch;

  EXPECT_TRUE(latch.accept(100));
  EXPECT_TRUE(latch.accept(100));
  latch.markCompleted(100);

  EXPECT_TRUE(latch.active());
  EXPECT_FALSE(latch.accept(100));
  EXPECT_TRUE(latch.active());
  EXPECT_TRUE(latch.accept(101));
  EXPECT_FALSE(latch.active());
  EXPECT_FALSE(latch.accept(100));
}

TEST(CompletedGoal, EmptyPathDoesNotReauthorizeTheCompletedGeneration)
{
  CompletedGoalLatch latch;
  latch.markCompleted(42);

  // An empty Path does not call clear(); the completed generation remains blocked.
  EXPECT_TRUE(latch.active());
  EXPECT_FALSE(latch.accept(42));

  latch.clear();
  EXPECT_FALSE(latch.active());
  EXPECT_TRUE(latch.accept(42));
}

TEST(MotionAuthorization, StartsDisarmedAndCanArmWhileWaitingForAPath)
{
  MotionAuthorization authorization;

  EXPECT_EQ(authorization.state(), MotionAuthorizationState::kDisarmed);
  EXPECT_FALSE(authorization.armed());
  EXPECT_FALSE(authorization.executionAuthorized());

  authorization.arm(false);

  EXPECT_EQ(authorization.state(), MotionAuthorizationState::kArmedWaitingForPath);
  EXPECT_TRUE(authorization.armed());
  EXPECT_FALSE(authorization.executionAuthorized());

  authorization.waitForPath();
  EXPECT_EQ(authorization.state(), MotionAuthorizationState::kArmedWaitingForPath);

  authorization.pathAvailable();

  EXPECT_EQ(authorization.state(), MotionAuthorizationState::kArmedExecuting);
  EXPECT_TRUE(authorization.executionAuthorized());
}

TEST(MotionAuthorization, ExpectedPathStopKeepsOneTimeAuthorization)
{
  MotionAuthorization authorization;
  authorization.arm(true);

  authorization.waitForPath();

  EXPECT_EQ(authorization.state(), MotionAuthorizationState::kArmedWaitingForPath);
  EXPECT_TRUE(authorization.armed());
  EXPECT_FALSE(authorization.executionAuthorized());

  authorization.pathAvailable();

  EXPECT_EQ(authorization.state(), MotionAuthorizationState::kArmedExecuting);
  EXPECT_TRUE(authorization.executionAuthorized());
}

TEST(MotionAuthorization, SafetyFaultDisarmsAndRejectsPathsUntilRearmed)
{
  MotionAuthorization authorization;
  authorization.arm(true);

  authorization.disarm();
  authorization.pathAvailable();

  EXPECT_EQ(authorization.state(), MotionAuthorizationState::kDisarmed);
  EXPECT_FALSE(authorization.armed());
  EXPECT_FALSE(authorization.executionAuthorized());
}

TEST(MotionAuthorization, PathTimeoutRequiresASecondExplicitArm)
{
  MotionAuthorization authorization;
  authorization.arm(true);

  authorization.disarm();
  authorization.pathAvailable();

  EXPECT_FALSE(authorization.armed());
  EXPECT_FALSE(authorization.executionAuthorized());

  authorization.arm(true);
  EXPECT_TRUE(authorization.executionAuthorized());
}

}  // namespace utree_go2_sdk2_bridge
