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
    20.0, 1.0, 0.5, 1.0, 0.2, 0.6, 0.15, 0.2,
    0.7853981633974483, 0.2617993877991494,
    0.05, 1.0, 1.5, 0.20, 2.0, 0.04, 0.05, 0.6, 0.35, 0.8};
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
  parameters.sport_state_timeout = 0.0;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.goal_yaw_tolerance = 4.0;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.heading_alignment_exit_angle = parameters.heading_alignment_enter_angle;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.explicit_rotation_tolerance = parameters.heading_alignment_exit_angle;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.max_vy = -0.1;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.minimum_translation_speed = 0.36;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.motion_response_timeout = 0.0;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.motion_response_min_translation = 0.5;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.max_yaw_rate = 4.0;
  parameters.motion_response_min_yaw = 3.2;
  EXPECT_FALSE(validateControlParameters(parameters).empty());
}

TEST(ControlParameters, AcceptsValuesWithinTheSdkCapabilityEnvelope)
{
  auto parameters = validParameters();
  parameters.max_vx = 2.5;
  parameters.max_vy = 1.0;
  parameters.max_yaw_rate = 4.0;
  EXPECT_TRUE(validateControlParameters(parameters).empty());
}

TEST(ControlParameters, RejectsValuesAboveTheSdkCapabilityEnvelope)
{
  auto parameters = validParameters();
  parameters.max_vx = 2.500001;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.max_vy = 1.000001;
  EXPECT_FALSE(validateControlParameters(parameters).empty());

  parameters = validParameters();
  parameters.max_yaw_rate = 4.000001;
  EXPECT_FALSE(validateControlParameters(parameters).empty());
}

TEST(MinimumPlanarSpeed, PreservesZeroAndCommandsAboveTheFloor)
{
  const VelocityCommand zero{0.0F, 0.0F, 0.2F};
  const auto preserved_zero = applyMinimumPlanarSpeed(zero, 0.20, 0.6, 0.35);
  ASSERT_TRUE(preserved_zero.has_value());
  EXPECT_FLOAT_EQ(preserved_zero->vx, 0.0F);
  EXPECT_FLOAT_EQ(preserved_zero->vy, 0.0F);
  EXPECT_FLOAT_EQ(preserved_zero->yaw_rate, 0.2F);

  const VelocityCommand already_fast{0.18F, -0.12F, -0.1F};
  const auto preserved_fast = applyMinimumPlanarSpeed(already_fast, 0.20, 0.6, 0.35);
  ASSERT_TRUE(preserved_fast.has_value());
  EXPECT_FLOAT_EQ(preserved_fast->vx, already_fast.vx);
  EXPECT_FLOAT_EQ(preserved_fast->vy, already_fast.vy);
  EXPECT_FLOAT_EQ(preserved_fast->yaw_rate, already_fast.yaw_rate);
}

TEST(MinimumPlanarSpeed, RaisesLowSpeedWithoutChangingDirection)
{
  const VelocityCommand low_speed{0.06F, -0.08F, 0.03F};
  const auto command = applyMinimumPlanarSpeed(low_speed, 0.20, 0.6, 0.35);
  ASSERT_TRUE(command.has_value());

  EXPECT_NEAR(command->vx, 0.12, 1.0e-6);
  EXPECT_NEAR(command->vy, -0.16, 1.0e-6);
  EXPECT_NEAR(std::hypot(command->vx, command->vy), 0.20, 1.0e-6);
  EXPECT_FLOAT_EQ(command->yaw_rate, low_speed.yaw_rate);
}

TEST(MinimumPlanarSpeed, RejectsInvalidOrUnreachableConfiguration)
{
  const VelocityCommand command{0.06F, -0.08F, 0.0F};
  EXPECT_FALSE(applyMinimumPlanarSpeed(command, 0.36, 0.6, 0.35).has_value());
  EXPECT_FALSE(applyMinimumPlanarSpeed(command, -0.1, 0.6, 0.35).has_value());

  auto invalid = command;
  invalid.vx = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(applyMinimumPlanarSpeed(invalid, 0.20, 0.6, 0.35).has_value());
}

TEST(MotionResponseWatchdog, TimesOutWithoutTranslationProgress)
{
  MotionResponseWatchdog watchdog;
  const VelocityCommand command{0.20F, 0.0F, 0.0F};

  const auto started = watchdog.observe(command, 0.0, 0.0, 0.0, 10.0, 2.0, 0.04, 0.05);
  ASSERT_TRUE(started.has_value());
  EXPECT_TRUE(*started);
  const auto waiting = watchdog.observe(command, 0.01, 0.0, 0.0, 11.9, 2.0, 0.04, 0.05);
  ASSERT_TRUE(waiting.has_value());
  EXPECT_TRUE(*waiting);
  const auto timed_out = watchdog.observe(command, 0.01, 0.0, 0.0, 12.01, 2.0, 0.04, 0.05);
  ASSERT_TRUE(timed_out.has_value());
  EXPECT_FALSE(*timed_out);
}

TEST(MotionResponseWatchdog, TranslationProgressRenewsTheDeadline)
{
  MotionResponseWatchdog watchdog;
  const VelocityCommand command{0.20F, 0.0F, 0.0F};

  const auto started = watchdog.observe(command, 0.0, 0.0, 0.0, 20.0, 2.0, 0.04, 0.05);
  ASSERT_TRUE(started.has_value());
  EXPECT_TRUE(*started);
  const auto progressed = watchdog.observe(command, 0.04, 0.0, 0.0, 21.9, 2.0, 0.04, 0.05);
  ASSERT_TRUE(progressed.has_value());
  EXPECT_TRUE(*progressed);
  const auto renewed = watchdog.observe(command, 0.04, 0.0, 0.0, 23.8, 2.0, 0.04, 0.05);
  ASSERT_TRUE(renewed.has_value());
  EXPECT_TRUE(*renewed);
  const auto timed_out = watchdog.observe(
    command, 0.04, 0.0, 0.0, 23.91, 2.0, 0.04, 0.05);
  ASSERT_TRUE(timed_out.has_value());
  EXPECT_FALSE(*timed_out);
}

TEST(MotionResponseWatchdog, HandlesWrappedYawAndZeroCommandReset)
{
  MotionResponseWatchdog watchdog;
  const VelocityCommand rotate{0.0F, 0.0F, 0.20F};
  const VelocityCommand zero{0.0F, 0.0F, 0.0F};

  const auto started = watchdog.observe(rotate, 0.0, 0.0, 3.13, 30.0, 2.0, 0.04, 0.05);
  ASSERT_TRUE(started.has_value());
  EXPECT_TRUE(*started);
  const auto wrapped_progress =
    watchdog.observe(rotate, 0.0, 0.0, -3.10, 31.0, 2.0, 0.04, 0.05);
  ASSERT_TRUE(wrapped_progress.has_value());
  EXPECT_TRUE(*wrapped_progress);
  const auto inactive = watchdog.observe(zero, 0.0, 0.0, -3.10, 34.0, 2.0, 0.04, 0.05);
  ASSERT_TRUE(inactive.has_value());
  EXPECT_TRUE(*inactive);
  const auto restarted = watchdog.observe(rotate, 0.0, 0.0, -3.10, 35.0, 2.0, 0.04, 0.05);
  ASSERT_TRUE(restarted.has_value());
  EXPECT_TRUE(*restarted);
}

TEST(MotionResponseWatchdog, RejectsNonFiniteOrBackwardSteadyTime)
{
  MotionResponseWatchdog watchdog;
  const VelocityCommand command{0.20F, 0.0F, 0.0F};
  const auto started = watchdog.observe(command, 0.0, 0.0, 0.0, 40.0, 2.0, 0.04, 0.05);
  ASSERT_TRUE(started.has_value());
  EXPECT_TRUE(*started);
  EXPECT_FALSE(watchdog.observe(command, 0.0, 0.0, 0.0, 39.9, 2.0, 0.04, 0.05).has_value());
  EXPECT_FALSE(
    watchdog.observe(
      command, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0,
      40.1, 2.0, 0.04, 0.05).has_value());
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

  const auto explicit_rotation = selectAlignmentYawError(
    0.25, 0.0, 0.10, 0.15, true);
  ASSERT_TRUE(explicit_rotation.has_value());
  EXPECT_DOUBLE_EQ(*explicit_rotation, 0.25);
}

TEST(HeadingAlignment, PendingExplicitRotationPreventsEarlyGoalCompletion)
{
  constexpr double snap_yaw = 0.19634954084936207;
  constexpr double explicit_tolerance = 0.05;
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0, 0.0),
    pathPose(0.0, 0.0, snap_yaw),
    pathPose(0.1, 0.0, snap_yaw)};
  PathProgressTracker tracker;

  const auto target = tracker.update(
    poses, 0.0, 0.0, 0.0, 0.3, explicit_tolerance);
  ASSERT_TRUE(target.has_value());
  ASSERT_TRUE(target->explicit_rotation_waypoint);

  const auto completion = goalCompletionReady(
    0.1, snap_yaw, 0.15, 0.20, target);
  ASSERT_TRUE(completion.has_value());
  EXPECT_FALSE(*completion);

  const std::optional<PathTrackingTarget> invalid_target;
  EXPECT_FALSE(goalCompletionReady(
      0.0, 0.0, 0.15, 0.20, invalid_target).has_value());

  const auto yaw_error = selectAlignmentYawError(
    snap_yaw, snap_yaw, 0.1, 0.15, target->explicit_rotation_waypoint);
  ASSERT_TRUE(yaw_error.has_value());
  EXPECT_DOUBLE_EQ(*yaw_error, snap_yaw);

  const auto rotate = requireRotateInPlace(
    false, target->explicit_rotation_waypoint, *yaw_error,
    explicit_tolerance, 0.1, 0.15);
  ASSERT_TRUE(rotate.has_value());
  ASSERT_TRUE(*rotate);

  const auto command = makeHeadingAwareCommand(
    0.1, 0.0, *yaw_error, *rotate, 0.6, 0.35, 0.8);
  ASSERT_TRUE(command.has_value());
  EXPECT_FLOAT_EQ(command->vx, 0.0F);
  EXPECT_FLOAT_EQ(command->vy, 0.0F);
  EXPECT_GT(command->yaw_rate, 0.0F);
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

TEST(PathProgress, PreservesASmallExplicitStartSnapRotation)
{
  constexpr double snap_yaw = 0.19634954084936207;
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(-0.1, -0.1, 0.0),
    pathPose(-0.1, -0.1, snap_yaw),
    pathPose(0.1, -0.1, snap_yaw)};
  PathProgressTracker tracker;

  const auto rotate = tracker.update(
    poses, -0.1, -0.1, 0.0, 0.3, 0.05);

  ASSERT_TRUE(rotate.has_value());
  EXPECT_TRUE(rotate->explicit_rotation_waypoint);
  EXPECT_EQ(rotate->heading_pose, 1U);
  EXPECT_NEAR(rotate->target_x, -0.1, 1.0e-12);
  EXPECT_NEAR(rotate->target_y, -0.1, 1.0e-12);

  const auto forward = tracker.update(
    poses, -0.1, -0.1, snap_yaw - 0.04, 0.3, 0.05);
  ASSERT_TRUE(forward.has_value());
  EXPECT_FALSE(forward->explicit_rotation_waypoint);
  EXPECT_EQ(forward->heading_pose, 1U);
}

TEST(PathProgress, ReportsPendingRotationAcrossAShortStartConnector)
{
  constexpr double snap_yaw = 0.19634954084936207;
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0, 0.0),
    pathPose(0.08, 0.0, 0.0),
    pathPose(0.08, 0.0, snap_yaw),
    pathPose(0.10, 0.0, snap_yaw)};
  PathProgressTracker tracker;

  const auto connector = tracker.update(poses, 0.0, 0.0, 0.0, 0.03, 0.05);
  ASSERT_TRUE(connector.has_value());
  EXPECT_FALSE(connector->explicit_rotation_waypoint);
  EXPECT_TRUE(connector->pending_explicit_rotation);
  const auto connector_yaw_error = selectAlignmentYawError(
    0.0, snap_yaw, 0.10, 0.15, connector->explicit_rotation_waypoint,
    connector->pending_explicit_rotation);
  ASSERT_TRUE(connector_yaw_error.has_value());
  EXPECT_DOUBLE_EQ(*connector_yaw_error, 0.0);
  const auto connector_rotate = requireRotateInPlace(
    false, connector->explicit_rotation_waypoint, *connector_yaw_error,
    0.05, 0.10, 0.15, connector->pending_explicit_rotation);
  ASSERT_TRUE(connector_rotate.has_value());
  EXPECT_FALSE(*connector_rotate);
  const auto connector_command = makeHeadingAwareCommand(
    connector->target_x, connector->target_y, *connector_yaw_error,
    *connector_rotate, 0.6, 0.35, 0.8);
  ASSERT_TRUE(connector_command.has_value());
  EXPECT_GT(connector_command->vx, 0.0F);

  const auto rotate = tracker.update(poses, 0.08, 0.0, 0.0, 0.03, 0.05);
  ASSERT_TRUE(rotate.has_value());
  EXPECT_TRUE(rotate->explicit_rotation_waypoint);
  EXPECT_TRUE(rotate->pending_explicit_rotation);

  const auto suffix = tracker.update(
    poses, 0.08, 0.0, snap_yaw - 0.04, 0.03, 0.05);
  ASSERT_TRUE(suffix.has_value());
  EXPECT_FALSE(suffix->explicit_rotation_waypoint);
  EXPECT_FALSE(suffix->pending_explicit_rotation);
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
  EXPECT_EQ(
    forward->translation_direction, PlannedTranslationDirection::kForward);
  const double body_dx = forward->target_y;
  ASSERT_GT(body_dx, 0.0);
  const auto filtered = filterLongitudinalCommand(
    body_dx, forward->translation_direction);
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
  EXPECT_FALSE(filterLongitudinalCommand(
      -0.02, PlannedTranslationDirection::kForward).has_value());

  const auto numerical_noise = filterLongitudinalCommand(
    -1.0e-5, PlannedTranslationDirection::kForward);
  ASSERT_TRUE(numerical_noise.has_value());
  EXPECT_DOUBLE_EQ(*numerical_noise, 0.0);

  const auto planned_reverse = filterLongitudinalCommand(
    -0.02, PlannedTranslationDirection::kReverse);
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
  EXPECT_EQ(
    target->translation_direction, PlannedTranslationDirection::kReverse);
  const auto command = filterLongitudinalCommand(
    target->target_x, target->translation_direction);
  ASSERT_TRUE(command.has_value());
  EXPECT_LT(*command, 0.0);
}

TEST(PathProgress, TreatsANearlyLateralSegmentWithoutLongitudinalMotion)
{
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pathPose(0.0, 0.0, 0.0), pathPose(-0.015, 0.20, 0.0)};
  PathProgressTracker tracker;

  const auto target = tracker.update(poses, 0.0, 0.0, 0.0, 0.3, 0.2617993877991494);

  ASSERT_TRUE(target.has_value());
  EXPECT_EQ(
    target->translation_direction, PlannedTranslationDirection::kLateral);
  const auto command = filterLongitudinalCommand(
    target->target_x, target->translation_direction);
  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(*command, 0.0);
}

TEST(PathProgress, KeepsSmallDirectionJitterAroundNinetyDegreesLateral)
{
  for (const double x : {-0.02, 0.02}) {
    const std::vector<geometry_msgs::msg::PoseStamped> poses{
      pathPose(0.0, 0.0, 0.0), pathPose(x, 0.20, 0.0)};
    PathProgressTracker tracker;

    const auto target = tracker.update(
      poses, 0.0, 0.0, 0.0, 0.3, 0.2617993877991494);

    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(
      target->translation_direction, PlannedTranslationDirection::kLateral);
    const auto command = filterLongitudinalCommand(
      target->target_x, target->translation_direction);
    ASSERT_TRUE(command.has_value());
    EXPECT_DOUBLE_EQ(*command, 0.0);
  }
}

TEST(PathProgress, PreservesLongitudinalMotionForRoundedDiagonalPrimitives)
{
  constexpr double yaw_bin = 3.1415926535897932 / 8.0;
  struct Case
  {
    double yaw;
    double target_y;
    PlannedTranslationDirection expected_direction;
  };
  for (const auto & test_case : std::vector<Case>{
      {yaw_bin, 0.20, PlannedTranslationDirection::kForward},
      {-yaw_bin, 0.20, PlannedTranslationDirection::kReverse},
      {yaw_bin, -0.20, PlannedTranslationDirection::kReverse},
      {-yaw_bin, -0.20, PlannedTranslationDirection::kForward}})
  {
    const std::vector<geometry_msgs::msg::PoseStamped> poses{
      pathPose(0.0, 0.0, test_case.yaw),
      pathPose(0.0, test_case.target_y, test_case.yaw)};
    PathProgressTracker tracker;

    const auto target = tracker.update(
      poses, 0.0, 0.0, test_case.yaw, 0.3, 0.2617993877991494);

    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(target->translation_direction, test_case.expected_direction);
    const double body_dx = std::sin(test_case.yaw) * target->target_y;
    const auto command = filterLongitudinalCommand(
      body_dx, target->translation_direction);
    ASSERT_TRUE(command.has_value());
    EXPECT_NE(*command, 0.0);
  }
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

  EXPECT_EQ(latch.evaluate(100), GoalGenerationDecision::kAccept);
  EXPECT_EQ(latch.evaluate(100), GoalGenerationDecision::kAccept);
  latch.markCompleted(100);

  EXPECT_TRUE(latch.active());
  EXPECT_EQ(latch.evaluate(100), GoalGenerationDecision::kCompletedReplay);
  EXPECT_TRUE(latch.active());
  EXPECT_EQ(latch.evaluate(101), GoalGenerationDecision::kAccept);
  EXPECT_FALSE(latch.active());
  EXPECT_EQ(latch.evaluate(100), GoalGenerationDecision::kSuperseded);
  EXPECT_EQ(latch.evaluate(0), GoalGenerationDecision::kInvalid);
}

TEST(SportStateSafety, AllowsOnlyTheObservedExecutableStates)
{
  EXPECT_TRUE(isExecutableSportState(100U));
  EXPECT_TRUE(isExecutableSportState(1013U));
  for (const std::uint32_t state_code : {1002U, 1015U, 2009U, 2011U, 9999U}) {
    EXPECT_FALSE(isExecutableSportState(state_code)) << "state_code=" << state_code;
  }
  EXPECT_STREQ(sportStateName(2009U), "jump run");
  EXPECT_STREQ(sportStateName(2011U), "handstand");
  EXPECT_STREQ(sportStateName(9999U), "unknown");
}

TEST(SportStateSafety, LatchesAnUnsafeTransientUntilConsumed)
{
  UnsafeSportStateLatch latch;
  latch.observe(SportStateSample{2011U, 0U, 0U});
  latch.observe(SportStateSample{100U, 0U, 0U});

  const auto unsafe = latch.take();
  ASSERT_TRUE(unsafe.has_value());
  EXPECT_EQ(unsafe->state_code, 2011U);
  EXPECT_FALSE(latch.take().has_value());
}

TEST(CompletedGoal, EmptyPathDoesNotReauthorizeTheCompletedGeneration)
{
  CompletedGoalLatch latch;
  latch.markCompleted(42);

  // An empty Path does not call clear(); the completed generation remains blocked.
  EXPECT_TRUE(latch.active());
  EXPECT_EQ(latch.evaluate(42), GoalGenerationDecision::kCompletedReplay);

  latch.clear();
  EXPECT_FALSE(latch.active());
  EXPECT_EQ(latch.evaluate(42), GoalGenerationDecision::kAccept);
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
