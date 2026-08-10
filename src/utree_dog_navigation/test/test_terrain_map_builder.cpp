#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

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

TEST(FlatObstacleMapBuilder, MarksOneInBandHitAndTreatsEveryOtherCellAsFlatFree)
{
  TerrainMapConfig map_config;
  map_config.resolution = 1.0;
  map_config.size_x = 3.0;
  map_config.size_y = 3.0;
  map_config.origin_x = 0.0;
  map_config.origin_y = 0.0;
  FlatObstacleLayerConfig obstacle_config;
  FlatObstacleMapBuilder builder(map_config, obstacle_config);

  EXPECT_EQ(
    builder.integrateFrame(
      {{1.5, 1.5, 10.10}, {1.5, 1.5, 10.00},
        {0.5, 0.5, 10.07}, {2.5, 2.5, 10.81}},
      1.0, 10.0, false),
    1U);

  const auto raw = builder.rawObstacleMask();
  ASSERT_EQ(raw.size(), 9U);
  EXPECT_EQ(raw[4], 1U);
  EXPECT_EQ(raw[0], 0U);
  EXPECT_EQ(raw[8], 0U);

  const auto map = builder.build(builtin_interfaces::msg::Time{}, "world", 10.0);
  ASSERT_EQ(map.traversability.size(), 9U);
  EXPECT_FLOAT_EQ(map.traversability[4], 0.0F);
  EXPECT_FLOAT_EQ(map.traversability[0], 1.0F);
  EXPECT_FLOAT_EQ(map.traversability[8], 1.0F);
  EXPECT_FLOAT_EQ(map.elevation[0], 10.0F);
  EXPECT_FLOAT_EQ(map.slope[0], 0.0F);
  EXPECT_FLOAT_EQ(map.roughness[0], 0.0F);
}

TEST(FlatObstacleMapBuilder, DoesNotClearWithoutSameCellGroundEndpointEvidence)
{
  TerrainMapConfig map_config;
  map_config.resolution = 1.0;
  map_config.size_x = 2.0;
  map_config.size_y = 1.0;
  map_config.origin_x = 0.0;
  map_config.origin_y = 0.0;
  FlatObstacleLayerConfig obstacle_config;
  obstacle_config.clear_after = 1.0;
  FlatObstacleMapBuilder builder(map_config, obstacle_config);

  builder.integrateFrame({{0.5, 0.5, 0.2}}, 0.0, 0.0, false);
  builder.integrateFrame({}, 0.6, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
  builder.integrateFrame({{1.5, 0.5, 0.0}}, 1.2, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
  builder.integrateFrame({{0.5, 0.5, 1.0}}, 2.2, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
  builder.integrateFrame({{0.5, 0.5, 0.0}}, 3.2, 0.0, false);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
}

TEST(FlatObstacleMapBuilder, ClearsOnlyAfterContinuousFreshSameCellGroundEndpoints)
{
  TerrainMapConfig map_config;
  map_config.resolution = 1.0;
  map_config.size_x = 1.0;
  map_config.size_y = 1.0;
  map_config.origin_x = 0.0;
  map_config.origin_y = 0.0;
  FlatObstacleLayerConfig obstacle_config;
  obstacle_config.clear_after = 1.0;
  FlatObstacleMapBuilder builder(map_config, obstacle_config);

  builder.integrateFrame({{0.5, 0.5, 0.2}}, 0.0, 0.0, false);
  builder.integrateFrame({{0.5, 0.5, 0.0}}, 0.6, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
  builder.integrateFrame({}, 0.7, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
  builder.integrateFrame({{0.5, 0.5, 0.0}}, 1.3, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
  builder.integrateFrame({{0.5, 0.5, 0.0}}, 1.7, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 0U);

  builder.integrateFrame({{0.5, 0.5, 0.2}}, 2.0, 0.0, true);
  builder.integrateFrame({{0.5, 0.5, 0.0}}, 5.0, 0.0, false);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
  builder.integrateFrame({{0.5, 0.5, 0.0}}, 5.6, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 1U);
  builder.integrateFrame({{0.5, 0.5, 0.0}}, 6.0, 0.0, true);
  EXPECT_EQ(builder.rawObstacleMask()[0], 0U);
}

TEST(FlatObstacleMapBuilder, InflatesClearanceByWholeGridCells)
{
  TerrainMapConfig map_config;
  map_config.resolution = 0.2;
  map_config.size_x = 1.0;
  map_config.size_y = 1.0;
  map_config.origin_x = 0.0;
  map_config.origin_y = 0.0;
  FlatObstacleLayerConfig obstacle_config;
  obstacle_config.clearance = 0.1;
  FlatObstacleMapBuilder builder(map_config, obstacle_config);

  builder.integrateFrame({{0.5, 0.5, 0.2}}, 1.0, 0.0, false);
  const auto inflated = builder.inflatedObstacleMask();
  EXPECT_EQ(
    std::count(inflated.begin(), inflated.end(), static_cast<std::uint8_t>(1U)), 9);
  for (std::size_t y = 1; y <= 3; ++y) {
    for (std::size_t x = 1; x <= 3; ++x) {
      EXPECT_EQ(inflated[y * 5 + x], 1U);
    }
  }
  EXPECT_EQ(inflated[0], 0U);
}

}  // namespace utree_dog_navigation
