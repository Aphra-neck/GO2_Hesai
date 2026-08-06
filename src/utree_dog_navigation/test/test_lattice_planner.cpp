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

}  // namespace utree_dog_navigation
