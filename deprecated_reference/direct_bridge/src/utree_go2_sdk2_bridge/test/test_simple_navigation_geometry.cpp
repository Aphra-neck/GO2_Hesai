#include <gtest/gtest.h>

#include "utree_go2_sdk2_bridge/simple_navigation_geometry.hpp"

namespace utree_go2_sdk2_bridge
{

TEST(SimpleNavigationGeometry, ChoosesFirstLegNearCurrentHeading)
{
  const auto route = makeRightAngleRoute(0.0, 0.0, 0.0, 2.0, 1.0, 0.05);
  ASSERT_EQ(route.size(), 2U);
  EXPECT_DOUBLE_EQ(route[0].x, 2.0);
  EXPECT_DOUBLE_EQ(route[0].y, 0.0);
  EXPECT_DOUBLE_EQ(route[1].x, 2.0);
  EXPECT_DOUBLE_EQ(route[1].y, 1.0);
}

TEST(SimpleNavigationGeometry, ChoosesVerticalFirstLegWhenFacingIt)
{
  constexpr double kHalfPi = 1.5707963267948966;
  const auto route = makeRightAngleRoute(0.0, 0.0, kHalfPi, 2.0, 1.0, 0.05);
  ASSERT_EQ(route.size(), 2U);
  EXPECT_DOUBLE_EQ(route[0].x, 0.0);
  EXPECT_DOUBLE_EQ(route[0].y, 1.0);
}

TEST(SimpleNavigationGeometry, CollapsesAxisAlignedGoal)
{
  const auto route = makeRightAngleRoute(0.0, 0.0, 0.0, 0.0, 1.0, 0.05);
  ASSERT_EQ(route.size(), 1U);
  EXPECT_DOUBLE_EQ(route[0].x, 0.0);
  EXPECT_DOUBLE_EQ(route[0].y, 1.0);
}

TEST(SimpleNavigationGeometry, EmptyWhenAlreadyAtGoal)
{
  EXPECT_TRUE(makeRightAngleRoute(1.0, 2.0, 0.0, 1.05, 2.0, 0.10).empty());
}

TEST(SimpleNavigationGeometry, ProducesReverseCorrectionAfterPassingTarget)
{
  const auto delta = targetDeltaInBody(1.25, 0.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(delta.x, -0.25, 1.0e-12);
  EXPECT_NEAR(delta.y, 0.0, 1.0e-12);
}

TEST(SimpleNavigationGeometry, RotatesWorldTargetDeltaIntoBodyAxes)
{
  constexpr double kHalfPi = 1.5707963267948966;
  const auto delta = targetDeltaInBody(0.0, 0.0, kHalfPi, 0.0, 1.0);
  EXPECT_NEAR(delta.x, 1.0, 1.0e-12);
  EXPECT_NEAR(delta.y, 0.0, 1.0e-12);
}

}  // namespace utree_go2_sdk2_bridge
