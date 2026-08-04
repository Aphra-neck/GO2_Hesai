#include <gtest/gtest.h>

#include <memory>

#include "utree_dog_navigation/lattice_planner.hpp"

namespace utree_dog_navigation
{

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

}  // namespace utree_dog_navigation
