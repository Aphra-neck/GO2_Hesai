#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include "utree_dog_navigation/lattice_planner.hpp"

namespace utree_dog_navigation
{
namespace
{

utree_dog_msgs::msg::TerrainGrid::SharedPtr makeSparseMap(float resolution)
{
  auto map = std::make_shared<utree_dog_msgs::msg::TerrainGrid>();
  map->resolution = resolution;
  map->width = 8;
  map->height = 8;
  map->origin_x = 0.0F;
  map->origin_y = 0.0F;
  map->unknown_value = -1000.0F;
  const std::size_t cell_count = map->width * map->height;
  map->elevation.assign(cell_count, map->unknown_value);
  map->slope.assign(cell_count, map->unknown_value);
  map->traversability.assign(cell_count, map->unknown_value);
  map->observation_count.assign(cell_count, 0U);
  return map;
}

void makeCellValid(
  const utree_dog_msgs::msg::TerrainGrid::SharedPtr & map, std::size_t x, std::size_t y)
{
  const std::size_t index = y * map->width + x;
  map->elevation[index] = 0.0F;
  map->slope[index] = 0.0F;
  map->traversability[index] = 1.0F;
  if (map->observation_count.size() == map->elevation.size()) {
    map->observation_count[index] = 4U;
  }
}

utree_dog_msgs::msg::TerrainGrid::SharedPtr makeStandingXt16BlindRingMap(
  int supported_sectors = 8)
{
  auto map = std::make_shared<utree_dog_msgs::msg::TerrainGrid>();
  map->resolution = 0.2F;
  map->width = 31;
  map->height = 31;
  map->origin_x = -3.1F;
  map->origin_y = -3.1F;
  map->unknown_value = -1000.0F;
  const std::size_t cell_count = map->width * map->height;
  map->elevation.assign(cell_count, map->unknown_value);
  map->slope.assign(cell_count, map->unknown_value);
  map->traversability.assign(cell_count, map->unknown_value);
  map->observation_count.assign(cell_count, 0U);

  constexpr double kPi = 3.14159265358979323846;
  for (std::size_t y = 0; y < map->height; ++y) {
    for (std::size_t x = 0; x < map->width; ++x) {
      const double world_x = map->origin_x + (static_cast<double>(x) + 0.5) * map->resolution;
      const double world_y = map->origin_y + (static_cast<double>(y) + 0.5) * map->resolution;
      const double radius = std::hypot(world_x, world_y);
      if (radius < 1.2 || radius > 2.4) {continue;}
      double angle = std::atan2(world_y, world_x);
      if (angle < 0.0) {angle += 2.0 * kPi;}
      const int sector = std::min(7, static_cast<int>(angle * 8.0 / (2.0 * kPi)));
      if (sector < supported_sectors) {
        makeCellValid(map, x, y);
      }
    }
  }
  return map;
}

utree_dog_msgs::msg::TerrainGrid::SharedPtr makeObservedToInferredReentryMap()
{
  auto map = std::make_shared<utree_dog_msgs::msg::TerrainGrid>();
  map->resolution = 0.2F;
  map->width = 41;
  map->height = 41;
  map->origin_x = -4.1F;
  map->origin_y = -4.1F;
  map->unknown_value = -1000.0F;
  const std::size_t cell_count = map->width * map->height;
  map->elevation.assign(cell_count, map->unknown_value);
  map->slope.assign(cell_count, map->unknown_value);
  map->traversability.assign(cell_count, map->unknown_value);
  map->observation_count.assign(cell_count, 0U);

  const auto set_observed = [&map](double world_x, double world_y, bool valid) {
      const int x = static_cast<int>(std::floor((world_x - map->origin_x) / map->resolution));
      const int y = static_cast<int>(std::floor((world_y - map->origin_y) / map->resolution));
      const std::size_t index = static_cast<std::size_t>(y) * map->width + x;
      map->elevation[index] = 0.0F;
      map->slope[index] = 0.0F;
      map->traversability[index] = valid ? 1.0F : 0.0F;
      map->observation_count[index] = 4U;
    };

  constexpr double kPi = 3.14159265358979323846;
  for (int sector = 0; sector < 8; ++sector) {
    const double angle = (static_cast<double>(sector) + 0.5) * 2.0 * kPi / 8.0;
    set_observed(-0.6 + 2.0 * std::cos(angle), 2.0 * std::sin(angle), true);
  }

  for (int step = -7; step <= 7; ++step) {
    set_observed(0.0, static_cast<double>(step) * 0.2, false);
  }
  for (int step = 0; step <= 9; ++step) {
    set_observed(-1.8, static_cast<double>(step) * 0.2, true);
  }
  for (int step = 0; step <= 11; ++step) {
    set_observed(-1.8 + static_cast<double>(step) * 0.2, 1.8, true);
  }
  for (int step = 4; step <= 9; ++step) {
    set_observed(0.4, static_cast<double>(step) * 0.2, true);
  }
  set_observed(0.4, -0.8, true);
  return map;
}

utree_dog_msgs::msg::TerrainGrid::SharedPtr makeFlatObstacleMap(
  float resolution, std::uint32_t width, std::uint32_t height)
{
  auto map = std::make_shared<utree_dog_msgs::msg::TerrainGrid>();
  map->resolution = resolution;
  map->width = width;
  map->height = height;
  map->origin_x = 0.0F;
  map->origin_y = 0.0F;
  map->unknown_value = -1000.0F;
  map->traversability.assign(
    static_cast<std::size_t>(width) * height, map->unknown_value);
  return map;
}

void markFlatObstacle(
  const utree_dog_msgs::msg::TerrainGrid::SharedPtr & map,
  std::size_t x, std::size_t y)
{
  map->traversability[y * map->width + x] = 0.0F;
}

}  // namespace

TEST(LatticePlanner, FindsPathAcrossFlatTraversableMap)
{
  auto map = std::make_shared<utree_dog_msgs::msg::TerrainGrid>();
  map->resolution = 1.0F;
  map->width = 10;
  map->height = 10;
  map->origin_x = 0.0F;
  map->origin_y = 0.0F;
  map->unknown_value = -1000.0F;
  const std::size_t cell_count = map->width * map->height;
  map->elevation.assign(cell_count, 0.0F);
  map->slope.assign(cell_count, 0.0F);
  map->traversability.assign(cell_count, 1.0F);

  LatticePlannerConfig config;
  config.yaw_bins = 8;
  config.motion_step = 1.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({1.5, 1.5, 0.0}, {6.5, 1.5, 0.0});
  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.states.empty());
  EXPECT_EQ(result.states.front().x, 1);
  EXPECT_EQ(result.states.back().x, 6);
}

TEST(LatticePlanner, FlatObstacleModeTreatsUnknownAsFreeAndUsesFixedElevation)
{
  auto map = makeFlatObstacleMap(0.1F, 50, 30);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  config.flat_obstacle.surface_elevation = 0.42;
  LatticePlanner planner(config);
  planner.setMap(map);

  ASSERT_TRUE(planner.mapValid());
  const auto result = planner.plan({1.05, 1.05, 0.0}, {2.05, 1.05, 0.0});

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.states.empty());
  EXPECT_EQ(result.start_status, VerifiedFlatStartStatus::kNotNeeded);
  EXPECT_TRUE(result.include_exact_start);
  EXPECT_DOUBLE_EQ(result.exact_start_elevation, 0.42);
  for (const auto & state : result.states) {
    EXPECT_FALSE(state.inferred);
    EXPECT_DOUBLE_EQ(state.elevation, 0.42);
    EXPECT_DOUBLE_EQ(state.dzdx, 0.0);
    EXPECT_DOUBLE_EQ(state.dzdy, 0.0);
  }
}

TEST(LatticePlanner, FlatObstacleModeDoesNotUndercostRoundedDiagonalStep)
{
  constexpr double kPi = 3.14159265358979323846;
  auto map = makeFlatObstacleMap(0.2F, 30, 30);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.yaw_bins = 16;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  config.flat_obstacle.footprint_length = 0.1;
  config.flat_obstacle.footprint_width = 0.1;
  config.flat_obstacle.obstacle_clearance = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({2.1, 2.1, 0.25 * kPi}, {2.3, 2.3, 0.25 * kPi});

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.states.size(), 2U);
  constexpr double kActualDiagonalDistance = 0.28284271247461901;
  EXPECT_GE(result.path_cost, kActualDiagonalDistance);
}

TEST(LatticePlanner, FlatObstacleModeRejectsTranslationThatRoundsToNoMovement)
{
  auto map = makeFlatObstacleMap(1.0F, 8, 8);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.yaw_bins = 8;
  config.motion_step = 0.1;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  config.max_expansions = 100;
  config.flat_obstacle.footprint_length = 0.2;
  config.flat_obstacle.footprint_width = 0.2;
  config.flat_obstacle.obstacle_clearance = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({2.5, 2.5, 0.0}, {3.5, 2.5, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_LE(result.expansions, config.yaw_bins);
}

TEST(LatticePlanner, FlatObstacleModeTurnsTowardClearPathBeforeTranslating)
{
  constexpr double kPi = 3.14159265358979323846;
  auto map = makeFlatObstacleMap(0.2F, 40, 30);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.yaw_bins = 16;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({2.1, 2.1, kPi}, {4.1, 2.1, kPi});

  ASSERT_TRUE(result.success);
  ASSERT_GE(result.states.size(), 3U);
  std::size_t first_translation = 1U;
  while (first_translation < result.states.size() &&
    result.states[first_translation].x == result.states[first_translation - 1U].x &&
    result.states[first_translation].y == result.states[first_translation - 1U].y)
  {
    ++first_translation;
  }
  ASSERT_LT(first_translation, result.states.size());
  EXPECT_GT(first_translation, 1U);

  const auto & before = result.states[first_translation - 1U];
  const auto & after = result.states[first_translation];
  const double travel_yaw = std::atan2(
    static_cast<double>(after.y - before.y),
    static_cast<double>(after.x - before.x));
  const double body_yaw = planner.yawAngle(before.yaw);
  const double heading_error = std::abs(std::remainder(travel_yaw - body_yaw, 2.0 * kPi));
  EXPECT_NEAR(heading_error, 0.0, 1.0e-9);
}

TEST(LatticePlanner, FlatObstacleModeFacesNextSegmentAtRightAngleCorner)
{
  constexpr double kPi = 3.14159265358979323846;
  auto map = makeFlatObstacleMap(1.0F, 10, 8);
  std::fill(map->traversability.begin(), map->traversability.end(), 0.0F);
  const auto clear_cell = [&map](std::size_t x, std::size_t y) {
      map->traversability[y * map->width + x] = map->unknown_value;
    };
  for (std::size_t x = 2; x <= 6; ++x) {clear_cell(x, 2);}
  for (std::size_t y = 2; y <= 4; ++y) {clear_cell(6, y);}

  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.yaw_bins = 8;
  config.motion_step = 1.0;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  config.flat_obstacle.footprint_length = 0.2;
  config.flat_obstacle.footprint_width = 0.2;
  config.flat_obstacle.obstacle_clearance = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({2.5, 2.5, 0.0}, {6.5, 4.5, 0.0});

  ASSERT_TRUE(result.success);
  auto first_north = result.states.end();
  for (auto state = result.states.begin() + 1; state != result.states.end(); ++state) {
    const auto & before = *(state - 1);
    if (state->y > before.y) {
      first_north = state;
      break;
    }
  }
  ASSERT_NE(first_north, result.states.end());
  const auto & before_north = *(first_north - 1);
  EXPECT_EQ(before_north.x, 6);
  EXPECT_EQ(before_north.y, 2);
  EXPECT_NEAR(planner.yawAngle(before_north.yaw), 0.5 * kPi, 1.0e-9);

  bool rotated_at_corner = false;
  for (auto state = result.states.begin(); state != first_north - 1; ++state) {
    if (state->x == before_north.x && state->y == before_north.y &&
      state->yaw != before_north.yaw)
    {
      rotated_at_corner = true;
      break;
    }
  }
  EXPECT_TRUE(rotated_at_corner);
}

TEST(LatticePlanner, FlatObstacleModeRetainsOneStepOmnidirectionalAdjustment)
{
  constexpr double kPi = 3.14159265358979323846;
  auto map = makeFlatObstacleMap(1.0F, 8, 8);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.yaw_bins = 8;
  config.motion_step = 1.0;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  config.flat_obstacle.footprint_length = 0.2;
  config.flat_obstacle.footprint_width = 0.2;
  config.flat_obstacle.obstacle_clearance = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({2.5, 2.5, 0.0}, {2.5, 3.5, 0.0});

  ASSERT_TRUE(result.success);
  std::size_t translation_count = 0U;
  double translation_heading_error = 0.0;
  for (std::size_t index = 1; index < result.states.size(); ++index) {
    const auto & before = result.states[index - 1U];
    const auto & after = result.states[index];
    if (before.x == after.x && before.y == after.y) {continue;}
    ++translation_count;
    const double travel_yaw = std::atan2(
      static_cast<double>(after.y - before.y),
      static_cast<double>(after.x - before.x));
    const double body_yaw = planner.yawAngle(before.yaw);
    translation_heading_error =
      std::abs(std::remainder(travel_yaw - body_yaw, 2.0 * kPi));
  }
  EXPECT_EQ(translation_count, 1U);
  EXPECT_GT(translation_heading_error, 1.0e-9);
  EXPECT_LE(translation_heading_error, 0.5 * kPi);
}

TEST(LatticePlanner, FlatObstacleModeAppliesFootprintClearanceAtEndpoints)
{
  auto map = makeFlatObstacleMap(0.05F, 80, 80);
  markFlatObstacle(map, 40, 47);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.start_snap_radius = 1.0;
  config.snap_radius = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({2.025, 2.025, 0.0}, {2.025, 2.025, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.expansions, 0);
}

TEST(LatticePlanner, FlatObstacleModeValidatesExactStartPoseBeforeGridCenter)
{
  auto map = makeFlatObstacleMap(1.0F, 6, 6);
  markFlatObstacle(map, 0, 1);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.flat_obstacle.footprint_length = 0.02;
  config.flat_obstacle.footprint_width = 0.02;
  config.flat_obstacle.obstacle_clearance = 0.0;
  config.start_snap_radius = 2.0;
  config.snap_radius = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({1.001, 1.5, 0.0}, {1.5, 1.5, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.expansions, 0);
  EXPECT_FALSE(result.include_exact_start);
}

TEST(LatticePlanner, FlatObstacleModeChecksCompleteLongitudinalSweep)
{
  auto clear_map = makeFlatObstacleMap(0.1F, 50, 9);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.motion_step = 2.0;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  config.max_expansions = 100;
  LatticePlanner clear_planner(config);
  clear_planner.setMap(clear_map);
  ASSERT_TRUE(clear_planner.plan({1.05, 0.45, 0.0}, {3.05, 0.45, 0.0}).success);

  auto blocked_map = makeFlatObstacleMap(0.1F, 50, 9);
  markFlatObstacle(blocked_map, 20, 4);
  LatticePlanner blocked_planner(config);
  blocked_planner.setMap(blocked_map);

  EXPECT_FALSE(blocked_planner.plan({1.05, 0.45, 0.0}, {3.05, 0.45, 0.0}).success);
}

TEST(LatticePlanner, FlatObstacleModeChecksCompleteLateralSweep)
{
  auto clear_map = makeFlatObstacleMap(0.1F, 13, 50);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.motion_step = 2.0;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  config.max_expansions = 100;
  LatticePlanner clear_planner(config);
  clear_planner.setMap(clear_map);
  ASSERT_TRUE(clear_planner.plan({0.65, 1.05, 0.0}, {0.65, 3.05, 0.0}).success);

  auto blocked_map = makeFlatObstacleMap(0.1F, 13, 50);
  markFlatObstacle(blocked_map, 6, 20);
  LatticePlanner blocked_planner(config);
  blocked_planner.setMap(blocked_map);

  EXPECT_FALSE(blocked_planner.plan({0.65, 1.05, 0.0}, {0.65, 3.05, 0.0}).success);
}

TEST(LatticePlanner, FlatObstacleModeChecksIntermediateRotationFootprint)
{
  constexpr double kQuarterTurn = 0.7853981633974483;
  auto clear_map = makeFlatObstacleMap(0.02F, 200, 200);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kFlatObstacle;
  config.yaw_bins = 8;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  config.max_expansions = 2;
  LatticePlanner clear_planner(config);
  clear_planner.setMap(clear_map);
  ASSERT_TRUE(clear_planner.plan({2.01, 2.01, 0.0}, {2.01, 2.01, kQuarterTurn}).success);

  auto blocked_map = makeFlatObstacleMap(0.02F, 200, 200);
  markFlatObstacle(blocked_map, 117, 126);
  LatticePlanner blocked_planner(config);
  blocked_planner.setMap(blocked_map);

  EXPECT_FALSE(
    blocked_planner.plan({2.01, 2.01, 0.0}, {2.01, 2.01, kQuarterTurn}).success);
}

TEST(LatticePlanner, TerrainModeStillRejectsUnknownTerrainCells)
{
  auto map = makeSparseMap(0.2F);
  LatticePlannerConfig config;
  config.planning_mode = PlanningMode::kTerrain;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  EXPECT_FALSE(planner.plan({0.3, 0.3, 0.0}, {0.3, 0.3, 0.0}).success);
}

TEST(LatticePlanner, RejectsNonFiniteMapMetadataAndLayers)
{
  auto map = makeSparseMap(0.2F);
  LatticePlanner planner(LatticePlannerConfig{});
  planner.setMap(map);
  ASSERT_TRUE(planner.mapValid());

  map->resolution = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(planner.mapValid());
  map->resolution = 0.2F;

  map->origin_x = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(planner.mapValid());
  map->origin_x = 0.0F;

  map->unknown_value = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(planner.mapValid());
  map->unknown_value = -1000.0F;

  map->elevation[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(planner.mapValid());
  map->elevation[0] = map->unknown_value;

  map->slope[0] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(planner.mapValid());
  map->slope[0] = map->unknown_value;

  map->traversability[0] = -std::numeric_limits<float>::infinity();
  EXPECT_FALSE(planner.mapValid());
}

TEST(LatticePlanner, RejectsOutOfRangeSlopeAndTraversability)
{
  auto map = makeSparseMap(0.2F);
  LatticePlanner planner(LatticePlannerConfig{});
  planner.setMap(map);

  ASSERT_TRUE(planner.mapValid());

  map->elevation[0] = -42.0F;
  EXPECT_TRUE(planner.mapValid());

  map->slope[0] = -0.01F;
  EXPECT_FALSE(planner.mapValid());
  map->slope[0] = map->unknown_value;

  map->traversability[0] = -0.01F;
  EXPECT_FALSE(planner.mapValid());
  map->traversability[0] = 1.01F;
  EXPECT_FALSE(planner.mapValid());
  map->traversability[0] = map->unknown_value;

  EXPECT_TRUE(planner.mapValid());
}

TEST(LatticePlanner, DoesNotPlanThroughCellWithUnknownElevation)
{
  auto map = makeSparseMap(0.2F);
  const std::size_t index = map->width + 1U;
  map->slope[index] = 0.0F;
  map->traversability[index] = 1.0F;

  LatticePlannerConfig config;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  ASSERT_TRUE(planner.mapValid());
  const auto result = planner.plan({0.3, 0.3, 0.0}, {0.3, 0.3, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.states.empty());
}

TEST(LatticePlanner, RejectsNonFiniteWorldStatesBeforeGridConversion)
{
  auto map = makeSparseMap(0.2F);
  makeCellValid(map, 1, 1);
  LatticePlanner planner(LatticePlannerConfig{});
  planner.setMap(map);

  const WorldState finite_state{0.3, 0.3, 0.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(planner.plan({nan, 0.3, 0.0}, finite_state).success);
  EXPECT_FALSE(planner.plan({0.3, infinity, 0.0}, finite_state).success);
  EXPECT_FALSE(planner.plan({0.3, 0.3, nan}, finite_state).success);
  EXPECT_FALSE(planner.plan(finite_state, {nan, 0.3, 0.0}).success);
  EXPECT_FALSE(planner.plan(finite_state, {0.3, infinity, 0.0}).success);
  EXPECT_FALSE(planner.plan(finite_state, {0.3, 0.3, nan}).success);
}

TEST(LatticePlanner, RejectsFiniteWorldStatesOutsideGridConversionRange)
{
  auto map = makeSparseMap(0.2F);
  makeCellValid(map, 1, 1);
  LatticePlanner planner(LatticePlannerConfig{});
  planner.setMap(map);

  const WorldState valid_state{0.3, 0.3, 0.0};
  const double maximum = std::numeric_limits<double>::max();
  EXPECT_FALSE(planner.plan({maximum, 0.3, 0.0}, valid_state).success);
  EXPECT_FALSE(planner.plan({-maximum, 0.3, 0.0}, valid_state).success);
  EXPECT_FALSE(planner.plan(valid_state, {0.3, maximum, 0.0}).success);
  EXPECT_FALSE(planner.plan(valid_state, {0.3, -maximum, 0.0}).success);
  EXPECT_TRUE(planner.plan({0.3, 0.3, maximum}, valid_state).success);
  EXPECT_TRUE(planner.plan({0.3, 0.3, -maximum}, valid_state).success);
}

TEST(LatticePlanner, RejectsUnsafeBaseConfigurationAtConstruction)
{
  LatticePlannerConfig config;
  config.motion_step = std::numeric_limits<double>::infinity();
  EXPECT_THROW(LatticePlanner planner(config), std::invalid_argument);

  config = LatticePlannerConfig{};
  config.motion_step = std::numeric_limits<double>::max();
  EXPECT_THROW(LatticePlanner planner(config), std::invalid_argument);

  config = LatticePlannerConfig{};
  config.snap_radius = std::numeric_limits<double>::max();
  EXPECT_THROW(LatticePlanner planner(config), std::invalid_argument);

  config = LatticePlannerConfig{};
  config.start_snap_radius = std::numeric_limits<double>::max();
  EXPECT_THROW(LatticePlanner planner(config), std::invalid_argument);

  config = LatticePlannerConfig{};
  config.max_step_height = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(LatticePlanner planner(config), std::invalid_argument);

  config = LatticePlannerConfig{};
  config.stair_height_threshold = config.max_step_height + 0.01;
  EXPECT_THROW(LatticePlanner planner(config), std::invalid_argument);
}

TEST(LatticePlanner, StopsSearchWhenCancellationIsRequested)
{
  auto map = std::make_shared<utree_dog_msgs::msg::TerrainGrid>();
  map->resolution = 0.2F;
  map->width = 40;
  map->height = 40;
  map->origin_x = 0.0F;
  map->origin_y = 0.0F;
  map->unknown_value = -1000.0F;
  const std::size_t cell_count = map->width * map->height;
  map->elevation.assign(cell_count, 0.0F);
  map->slope.assign(cell_count, 0.0F);
  map->traversability.assign(cell_count, 1.0F);

  LatticePlannerConfig config;
  config.motion_step = 0.2;
  LatticePlanner planner(config);
  planner.setMap(map);

  int cancellation_checks = 0;
  const auto result = planner.plan(
    {0.3, 0.3, 0.0}, {7.5, 7.5, 0.0},
    [&cancellation_checks]() {return ++cancellation_checks >= 3;});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(cancellation_checks, 3);
  EXPECT_LT(result.expansions, config.max_expansions);
}

TEST(LatticePlanner, RejectsSnapCandidateOutsideEuclideanRadius)
{
  auto map = makeSparseMap(0.2F);
  makeCellValid(map, 2, 4);

  LatticePlannerConfig config;
  config.snap_radius = 0.5;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.3, 0.3, 0.0}, {0.5, 0.9, 0.0});
  EXPECT_FALSE(result.success);
}

TEST(LatticePlanner, MapBoundsSafelyLimitVeryLargeFiniteSnapRadius)
{
  auto map = makeSparseMap(0.2F);
  makeCellValid(map, 7, 7);

  LatticePlannerConfig config;
  config.snap_radius = static_cast<double>(std::numeric_limits<int>::max());
  config.start_snap_radius = config.snap_radius;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.1, 0.1, 0.0}, {0.1, 0.1, 0.0});

  EXPECT_TRUE(result.success);
  ASSERT_FALSE(result.states.empty());
  EXPECT_EQ(result.states.front().x, 7);
  EXPECT_EQ(result.states.front().y, 7);
}

TEST(LatticePlanner, AcceptsSnapCandidateOnEuclideanRadiusBoundary)
{
  auto map = makeSparseMap(0.1F);
  makeCellValid(map, 1, 6);

  LatticePlannerConfig config;
  config.snap_radius = 0.5;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.15, 0.15, 0.0}, {0.15, 0.65, 0.0});
  EXPECT_TRUE(result.success);
}

TEST(LatticePlanner, SnapDistanceIsContinuousAcrossSourceCellBoundary)
{
  auto map = makeSparseMap(0.2F);
  makeCellValid(map, 1, 4);

  LatticePlannerConfig config;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  LatticePlanner planner(config);
  planner.setMap(map);

  const WorldState goal{0.3, 0.9, 0.0};
  const auto below_boundary = planner.plan({0.3, 0.399, 0.0}, goal);
  const auto above_boundary = planner.plan({0.3, 0.401, 0.0}, goal);

  EXPECT_TRUE(below_boundary.success);
  EXPECT_TRUE(above_boundary.success);
  ASSERT_FALSE(below_boundary.states.empty());
  ASSERT_FALSE(above_boundary.states.empty());
  EXPECT_EQ(below_boundary.states.front().x, above_boundary.states.front().x);
  EXPECT_EQ(below_boundary.states.front().y, above_boundary.states.front().y);
}

TEST(LatticePlanner, GoalKeepsItsNarrowerSnapRadius)
{
  auto map = makeSparseMap(0.2F);
  makeCellValid(map, 1, 4);

  LatticePlannerConfig config;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  LatticePlanner planner(config);
  planner.setMap(map);

  const WorldState valid_start{0.3, 0.9, 0.0};
  const auto result = planner.plan(valid_start, {0.3, 0.399, 0.0});

  EXPECT_FALSE(result.success);
}

TEST(LatticePlanner, LegacySnapRadiusAlsoControlsStart)
{
  auto map = makeSparseMap(0.1F);
  makeCellValid(map, 1, 3);

  // Keep the original aggregate field order working when the new field is omitted.
  LatticePlannerConfig config{
    16, 0.20, 0.18, 0.24, 0.65, 0.08, 4.0, 1.5, 2.0, 0.15, 1.15, 1.25, 250000,
    0.1};
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.15, 0.251, 0.0}, {0.15, 0.35, 0.0});

  EXPECT_TRUE(result.success);
}

TEST(LatticePlanner, ExactValidCellDoesNotRequireSnapping)
{
  auto map = makeSparseMap(1.0F);
  makeCellValid(map, 0, 0);

  LatticePlannerConfig config;
  config.start_snap_radius = 0.0;
  config.snap_radius = 0.0;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.99, 0.5, 0.0}, {0.5, 0.5, 0.0});

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.states.empty());
  EXPECT_EQ(result.states.front().x, 0);
  EXPECT_EQ(result.states.front().y, 0);
}

TEST(LatticePlanner, VerifiedFlatBlindRingResolvesStandingXt16Start)
{
  auto map = makeStandingXt16BlindRingMap();
  const auto original_elevation = map->elevation;
  const auto original_slope = map->slope;
  const auto original_traversability = map->traversability;
  const auto original_observation_count = map->observation_count;

  LatticePlannerConfig config;
  config.yaw_bins = 8;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  config.verified_flat_start.enabled = true;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.0, 0.0, 0.0}, {1.4, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.start_status, VerifiedFlatStartStatus::kApplied);
  ASSERT_FALSE(result.states.empty());
  EXPECT_TRUE(result.states.front().inferred);
  EXPECT_FALSE(result.states.back().inferred);
  bool reached_measured = false;
  for (const auto & state : result.states) {
    if (!state.inferred) {reached_measured = true;}
    EXPECT_FALSE(reached_measured && state.inferred);
    EXPECT_TRUE(std::isfinite(state.elevation));
    EXPECT_TRUE(std::isfinite(state.dzdx));
    EXPECT_TRUE(std::isfinite(state.dzdy));
  }
  EXPECT_EQ(map->elevation, original_elevation);
  EXPECT_EQ(map->slope, original_slope);
  EXPECT_EQ(map->traversability, original_traversability);
  EXPECT_EQ(map->observation_count, original_observation_count);
}

TEST(LatticePlanner, VerifiedFlatBlindRingIsDisabledByDefault)
{
  auto map = makeStandingXt16BlindRingMap();
  LatticePlannerConfig config;
  config.yaw_bins = 8;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.0, 0.0, 0.0}, {1.4, 0.0, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.start_status, VerifiedFlatStartStatus::kDisabled);
}

TEST(LatticePlanner, RejectsInvalidVerifiedFlatConfigurationAtConstruction)
{
  LatticePlannerConfig enabled_config;
  enabled_config.verified_flat_start.enabled = true;
  enabled_config.verified_flat_start.support_outer_radius =
    enabled_config.verified_flat_start.support_inner_radius;
  EXPECT_THROW(LatticePlanner planner(enabled_config), std::invalid_argument);

  LatticePlannerConfig disabled_config = enabled_config;
  disabled_config.verified_flat_start.enabled = false;
  EXPECT_NO_THROW(LatticePlanner planner(disabled_config));
}

TEST(LatticePlanner, VerifiedFlatBlindRingRequiresBroadSectorSupport)
{
  auto map = makeStandingXt16BlindRingMap(5);
  LatticePlannerConfig config;
  config.yaw_bins = 8;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  config.verified_flat_start.enabled = true;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.0, 0.0, 0.0}, {1.4, 0.0, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.start_status, VerifiedFlatStartStatus::kInsufficientSectors);
}

TEST(LatticePlanner, VerifiedFlatBlindRingRequiresMotionConnectionToObservedTerrain)
{
  auto map = makeStandingXt16BlindRingMap();
  for (std::size_t y = 0; y < map->height; ++y) {
    for (std::size_t x = 0; x < map->width; ++x) {
      const double world_x = map->origin_x + (static_cast<double>(x) + 0.5) * map->resolution;
      const double world_y = map->origin_y + (static_cast<double>(y) + 0.5) * map->resolution;
      if (std::hypot(world_x, world_y) >= 1.8) {continue;}
      const std::size_t index = y * map->width + x;
      map->elevation[index] = map->unknown_value;
      map->slope[index] = map->unknown_value;
      map->traversability[index] = map->unknown_value;
      map->observation_count[index] = 0U;
    }
  }

  LatticePlannerConfig config;
  config.yaw_bins = 8;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  config.verified_flat_start.enabled = true;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.start_status, VerifiedFlatStartStatus::kNoObservedConnection);
  EXPECT_EQ(result.expansions, 0);
}

TEST(LatticePlanner, VerifiedFlatBlindRingNeverAcceptsAnInferredGoal)
{
  auto map = makeStandingXt16BlindRingMap();
  LatticePlannerConfig config;
  config.yaw_bins = 8;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  config.verified_flat_start.enabled = true;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.0, 0.0, 0.0}, {0.4, 0.0, 0.0});

  EXPECT_FALSE(result.success);
}

TEST(LatticePlanner, VerifiedFlatBlindRingRejectsNonPlanarSupport)
{
  auto map = makeStandingXt16BlindRingMap();
  constexpr double kPi = 3.14159265358979323846;
  for (std::size_t y = 0; y < map->height; ++y) {
    for (std::size_t x = 0; x < map->width; ++x) {
      const std::size_t index = y * map->width + x;
      if (map->elevation[index] == map->unknown_value) {continue;}
      const double world_x = map->origin_x + (static_cast<double>(x) + 0.5) * map->resolution;
      const double world_y = map->origin_y + (static_cast<double>(y) + 0.5) * map->resolution;
      double angle = std::atan2(world_y, world_x);
      if (angle < 0.0) {angle += 2.0 * kPi;}
      const int sector = std::min(7, static_cast<int>(angle * 8.0 / (2.0 * kPi)));
      map->elevation[index] = sector % 2 == 0 ? 0.0F : 0.16F;
    }
  }

  LatticePlannerConfig config;
  config.yaw_bins = 8;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  config.verified_flat_start.enabled = true;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({0.0, 0.0, 0.0}, {1.4, 0.0, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.start_status, VerifiedFlatStartStatus::kSupportNotFlat);
  EXPECT_TRUE(result.states.empty());
}

TEST(LatticePlanner, ExistingObservedStartBypassesVerifiedFlatOverlay)
{
  auto map = makeStandingXt16BlindRingMap();
  for (std::size_t x = 15; x <= 22; ++x) {makeCellValid(map, x, 15);}

  LatticePlannerConfig enabled_config;
  enabled_config.yaw_bins = 8;
  enabled_config.motion_step = 0.2;
  enabled_config.start_snap_radius = 0.55;
  enabled_config.snap_radius = 0.5;
  enabled_config.verified_flat_start.enabled = true;
  LatticePlanner enabled_planner(enabled_config);
  enabled_planner.setMap(map);

  LatticePlannerConfig disabled_config = enabled_config;
  disabled_config.verified_flat_start.enabled = false;
  LatticePlanner disabled_planner(disabled_config);
  disabled_planner.setMap(map);

  const auto enabled = enabled_planner.plan({0.0, 0.0, 0.0}, {1.4, 0.0, 0.0});
  const auto disabled = disabled_planner.plan({0.0, 0.0, 0.0}, {1.4, 0.0, 0.0});

  ASSERT_TRUE(enabled.success);
  ASSERT_TRUE(disabled.success);
  EXPECT_EQ(enabled.start_status, VerifiedFlatStartStatus::kNotNeeded);
  EXPECT_EQ(enabled.expansions, disabled.expansions);
  ASSERT_EQ(enabled.states.size(), disabled.states.size());
  for (std::size_t index = 0; index < enabled.states.size(); ++index) {
    EXPECT_EQ(enabled.states[index].x, disabled.states[index].x);
    EXPECT_EQ(enabled.states[index].y, disabled.states[index].y);
    EXPECT_EQ(enabled.states[index].yaw, disabled.states[index].yaw);
    EXPECT_FALSE(enabled.states[index].inferred);
    EXPECT_DOUBLE_EQ(enabled.states[index].elevation, disabled.states[index].elevation);
    EXPECT_DOUBLE_EQ(enabled.states[index].dzdx, disabled.states[index].dzdx);
    EXPECT_DOUBLE_EQ(enabled.states[index].dzdy, disabled.states[index].dzdy);
  }
}

TEST(LatticePlanner, CannotReenterInferredRegionAfterReachingObservedTerrain)
{
  auto map = makeObservedToInferredReentryMap();
  LatticePlannerConfig config;
  config.yaw_bins = 8;
  config.motion_step = 0.2;
  config.start_snap_radius = 0.55;
  config.snap_radius = 0.5;
  config.verified_flat_start.enabled = true;
  config.verified_flat_start.min_supported_sectors = 8;
  config.verified_flat_start.min_cells_per_sector = 1;
  config.verified_flat_start.min_support_cells = 8;
  LatticePlanner planner(config);
  planner.setMap(map);

  const auto result = planner.plan({-0.6, 0.0, 0.0}, {0.4, -0.8, 0.0});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.start_status, VerifiedFlatStartStatus::kApplied);
  EXPECT_GT(result.expansions, 0);
  EXPECT_TRUE(result.states.empty());
}

}  // namespace utree_dog_navigation
