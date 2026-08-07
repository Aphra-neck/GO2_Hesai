#include <gtest/gtest.h>

#include <memory>

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
  return map;
}

void makeCellValid(
  const utree_dog_msgs::msg::TerrainGrid::SharedPtr & map, std::size_t x, std::size_t y)
{
  const std::size_t index = y * map->width + x;
  map->elevation[index] = 0.0F;
  map->slope[index] = 0.0F;
  map->traversability[index] = 1.0F;
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

}  // namespace utree_dog_navigation
