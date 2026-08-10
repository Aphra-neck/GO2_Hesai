#include <gtest/gtest.h>

#include "builtin_interfaces/msg/time.hpp"
#include "utree_dog_navigation/terrain_map_builder.hpp"

namespace utree_dog_navigation
{

TEST(TerrainMapBuilder, FillsOnlySurroundedSingleCellHole)
{
  TerrainMapConfig config;
  config.resolution = 1.0;
  config.size_x = 3.0;
  config.size_y = 3.0;
  config.origin_x = 0.0;
  config.origin_y = 0.0;
  config.min_points_per_cell = 1;
  config.min_observed_frames = 1;
  config.max_slope = 1.0;
  config.max_roughness = 1.0;
  TerrainMapBuilder builder(config);

  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x) {
      if (x != 1 || y != 1) {
        ASSERT_TRUE(builder.addPoint(x + 0.5, y + 0.5, 0.0));
      }
    }
  }

  const auto map = builder.build(builtin_interfaces::msg::Time{}, "map");
  EXPECT_FLOAT_EQ(map.elevation[4], 0.0F);
  EXPECT_FLOAT_EQ(map.variance[4], 0.0F);
  EXPECT_FLOAT_EQ(map.slope[4], 0.0F);
  EXPECT_FLOAT_EQ(map.traversability[4], 1.0F);
}

TEST(TerrainMapBuilder, IntegratesSparseFramesAndExpiresOldObservations)
{
  TerrainMapConfig config;
  config.resolution = 0.1;
  config.size_x = 0.3;
  config.size_y = 0.3;
  config.origin_x = 0.0;
  config.origin_y = 0.0;
  config.min_observed_frames = 3;
  config.integration_window = 1.0;
  config.confidence_frames = 4.0;
  TerrainMapBuilder builder(config);

  for (int frame = 0; frame < 3; ++frame) {
    EXPECT_EQ(
      builder.integrateFrame({{0.15, 0.15, 0.20 + frame * 0.001}}, frame * 0.1), 1U);
  }
  EXPECT_EQ(builder.integrateFrame({{1.0, 1.0, 0.0}}, 0.3), 0U);
  auto map = builder.build(builtin_interfaces::msg::Time{}, "map");
  EXPECT_EQ(map.observation_count[4], 3);
  EXPECT_NEAR(map.elevation[4], 0.201, 1.0e-4);
  EXPECT_GT(map.confidence[4], 0.7F);

  EXPECT_EQ(builder.integrateFrame({}, 2.0), 0U);
  map = builder.build(builtin_interfaces::msg::Time{}, "map");
  EXPECT_EQ(map.observation_count[4], 0);
  EXPECT_FLOAT_EQ(map.elevation[4], TerrainMapBuilder::kUnknown);
}

}  // namespace utree_dog_navigation
