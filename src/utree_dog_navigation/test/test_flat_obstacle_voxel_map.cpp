#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "utree_dog_navigation/flat_obstacle_voxel_map.hpp"

namespace utree_dog_navigation
{
namespace
{
TerrainMapConfig mapConfig()
{
  TerrainMapConfig config;
  config.resolution = 0.20;
  config.size_x = 4.0;
  config.size_y = 4.0;
  config.origin_x = -2.0;
  config.origin_y = -2.0;
  return config;
}

FlatObstacleVoxelConfig voxelConfig()
{
  FlatObstacleVoxelConfig config;
  config.min_height = 0.08;
  config.max_height = 0.80;
  config.voxel_height = 0.10;
  config.clearance = 0.0;
  config.strong_hit_points = 3;
  config.hit_confirmation_frames = 2;
  config.hit_confirmation_window = 0.35;
  config.clear_confirmation_frames = 2;
  config.clear_confirmation_window = 0.35;
  return config;
}

std::size_t countOccupied(const FlatObstacleSnapshot & snapshot)
{
  return static_cast<std::size_t>(std::count(
      snapshot.raw_obstacles.begin(), snapshot.raw_obstacles.end(),
      static_cast<std::uint8_t>(1U)));
}
}  // namespace

TEST(FlatObstacleVoxelMap, SingleFrameOutlierDoesNotBecomeProjectedObstacle)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};

  const auto update = map.update(
    {{1.0, 0.0, 0.25}}, sensor_origin, 1.0, 0.0, true);
  const auto snapshot = map.snapshot(
    builtin_interfaces::msg::Time{}, "world", 0.0, true);

  ASSERT_TRUE(update.accepted);
  EXPECT_EQ(update.confirmed_voxels, 0U);
  EXPECT_EQ(countOccupied(snapshot), 0U);
  EXPECT_TRUE(snapshot.confirmed_voxel_centers.empty());
}

TEST(FlatObstacleVoxelMap, RepeatedThinObstacleIsConfirmedOnSecondFreshFrame)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};

  map.update({{1.0, 0.0, 0.25}}, sensor_origin, 1.0, 0.0, true);
  const auto update = map.update(
    {{1.0, 0.0, 0.25}}, sensor_origin, 1.1, 0.0, true);
  const auto snapshot = map.snapshot(
    builtin_interfaces::msg::Time{}, "world", 0.0, true);

  EXPECT_EQ(update.newly_confirmed_voxels, 1U);
  EXPECT_EQ(update.confirmed_voxels, 1U);
  EXPECT_EQ(countOccupied(snapshot), 1U);
  ASSERT_EQ(snapshot.confirmed_voxel_centers.size(), 1U);
  EXPECT_NEAR(snapshot.confirmed_voxel_centers.front().z, 0.23, 0.051);
}

TEST(FlatObstacleVoxelMap, DenseReturnConfirmsObstacleInOneFrame)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};

  const auto update = map.update(
    {{1.01, 0.01, 0.25}, {1.02, 0.02, 0.26}, {1.03, 0.03, 0.27}},
    sensor_origin, 1.0, 0.0, true);

  EXPECT_EQ(update.newly_confirmed_voxels, 1U);
  EXPECT_EQ(update.confirmed_voxels, 1U);
}

TEST(FlatObstacleVoxelMap, InflatesOnlyThe2DVisualizationAndCostmapSnapshot)
{
  auto config = voxelConfig();
  config.clearance = 0.10;
  FlatObstacleVoxelMap map(mapConfig(), config);
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};
  map.update(
    {{1.01, 0.01, 0.25}, {1.02, 0.02, 0.26}, {1.03, 0.03, 0.27}},
    sensor_origin, 1.0, 0.0, true);

  const auto snapshot = map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0);
  EXPECT_EQ(countOccupied(snapshot), 1U);
  EXPECT_EQ(
    std::count(
      snapshot.inflated_obstacles.begin(), snapshot.inflated_obstacles.end(),
      static_cast<std::uint8_t>(1U)),
    9);
  EXPECT_EQ(snapshot.terrain.traversability.size(), snapshot.raw_obstacles.size());
  EXPECT_EQ(
    std::count(
      snapshot.terrain.traversability.begin(), snapshot.terrain.traversability.end(), 0.0F),
    1);
}

TEST(FlatObstacleVoxelMap, TwoFreshFreeRaysClearConfirmedVoxel)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.25};
  map.update({{1.0, 0.0, 0.25}}, sensor_origin, 1.0, 0.0, true);
  map.update({{1.0, 0.0, 0.25}}, sensor_origin, 1.1, 0.0, true);
  ASSERT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 1U);

  auto update = map.update({{3.0, 0.0, 0.25}}, sensor_origin, 1.2, 0.0, true);
  EXPECT_EQ(update.cleared_voxels, 0U);
  EXPECT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 1U);

  update = map.update({{3.0, 0.0, 0.25}}, sensor_origin, 1.3, 0.0, true);
  EXPECT_EQ(update.cleared_voxels, 1U);
  EXPECT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 0U);
}

TEST(FlatObstacleVoxelMap, GroundReturnBehindRemovedObstacleClearsAfterTwoFrames)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};
  map.update({{1.0, 0.0, 0.25}}, sensor_origin, 1.0, 0.0, true);
  map.update({{1.0, 0.0, 0.25}}, sensor_origin, 1.1, 0.0, true);
  ASSERT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 1U);

  auto update = map.update({{1.8, 0.0, 0.0}}, sensor_origin, 1.2, 0.0, true);
  EXPECT_EQ(update.cleared_voxels, 0U);
  update = map.update({{1.8, 0.0, 0.0}}, sensor_origin, 1.3, 0.0, true);

  EXPECT_EQ(update.cleared_voxels, 1U);
  EXPECT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 0U);
}

TEST(FlatObstacleVoxelMap, RayAboveObstacleDoesNotClearLowerVoxel)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint low_origin{0.0, 0.0, 0.25};
  map.update({{1.0, 0.0, 0.25}}, low_origin, 1.0, 0.0, true);
  map.update({{1.0, 0.0, 0.25}}, low_origin, 1.1, 0.0, true);

  const TerrainPoint high_origin{0.0, 0.0, 0.65};
  map.update({{3.0, 0.0, 0.65}}, high_origin, 1.2, 0.0, true);
  map.update({{3.0, 0.0, 0.65}}, high_origin, 1.3, 0.0, true);

  EXPECT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 1U);
}

TEST(FlatObstacleVoxelMap, ReturnAboveCollisionEnvelopeDoesNotProjectTo2D)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};

  map.update(
    {{1.01, 0.01, 0.90}, {1.02, 0.02, 0.91}, {1.03, 0.03, 0.92}},
    sensor_origin, 1.0, 0.0, true);

  const auto snapshot = map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0);
  EXPECT_EQ(countOccupied(snapshot), 0U);
  EXPECT_TRUE(snapshot.confirmed_voxel_centers.empty());
}

TEST(FlatObstacleVoxelMap, ObstacleEndpointWinsOverFreeRayInSameFrame)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.25};

  const auto update = map.update(
    {{1.01, 0.01, 0.25}, {1.02, 0.02, 0.26}, {1.03, 0.03, 0.27},
      {1.8, 0.0, 0.25}},
    sensor_origin, 1.0, 0.0, true);

  EXPECT_EQ(update.newly_confirmed_voxels, 1U);
  EXPECT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 1U);
}

TEST(FlatObstacleVoxelMap, NonmonotonicFrameCannotConfirmCandidate)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};
  map.update({{1.0, 0.0, 0.25}}, sensor_origin, 1.0, 0.0, true);

  const auto duplicate = map.update(
    {{1.0, 0.0, 0.25}}, sensor_origin, 1.0, 0.0, true);

  EXPECT_FALSE(duplicate.accepted);
  EXPECT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 0U);
}

TEST(FlatObstacleVoxelMap, DiscontinuousTimingStartsANewOccupancyEpoch)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};
  map.update(
    {{1.01, 0.01, 0.25}, {1.02, 0.02, 0.26}, {1.03, 0.03, 0.27}},
    sensor_origin, 1.0, 0.0, true);
  ASSERT_EQ(
    countOccupied(map.snapshot(builtin_interfaces::msg::Time{}, "world", 0.0)), 1U);

  const auto update = map.update(
    {{-1.01, 0.01, 0.25}, {-1.02, 0.02, 0.26}, {-1.03, 0.03, 0.27}},
    sensor_origin, 2.0, 0.0, false);
  const auto snapshot = map.snapshot(
    builtin_interfaces::msg::Time{}, "world", 0.0, true);

  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.epoch_reset);
  ASSERT_EQ(snapshot.confirmed_voxel_centers.size(), 1U);
  EXPECT_LT(snapshot.confirmed_voxel_centers.front().x, 0.0);
}

TEST(FlatObstacleVoxelMap, BackwardClockResetStartsANewOccupancyEpoch)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};
  map.update(
    {{1.01, 0.01, 0.25}, {1.02, 0.02, 0.26}, {1.03, 0.03, 0.27}},
    sensor_origin, 2.0, 0.0, true);

  const auto update = map.update(
    {{-1.01, 0.01, 0.25}, {-1.02, 0.02, 0.26}, {-1.03, 0.03, 0.27}},
    sensor_origin, 1.0, 0.0, true);
  const auto snapshot = map.snapshot(
    builtin_interfaces::msg::Time{}, "world", 0.0, true);

  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.epoch_reset);
  ASSERT_EQ(snapshot.confirmed_voxel_centers.size(), 1U);
  EXPECT_LT(snapshot.confirmed_voxel_centers.front().x, 0.0);
}

TEST(FlatObstacleVoxelMap, SnapshotBuildsVoxelCentersOnlyWhenRequested)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};
  map.update(
    {{1.01, 0.01, 0.25}, {1.02, 0.02, 0.26}, {1.03, 0.03, 0.27}},
    sensor_origin, 1.0, 0.0, true);

  const auto planner_only = map.snapshot(
    builtin_interfaces::msg::Time{}, "world", 0.0, false);
  const auto with_visualization = map.snapshot(
    builtin_interfaces::msg::Time{}, "world", 0.0, true);

  EXPECT_EQ(countOccupied(planner_only), 1U);
  EXPECT_TRUE(planner_only.confirmed_voxel_centers.empty());
  EXPECT_EQ(with_visualization.confirmed_voxel_centers.size(), 1U);
}

TEST(FlatObstacleVoxelMap, TopVoxelCenterStaysInsideTruncatedHeightSlice)
{
  FlatObstacleVoxelMap map(mapConfig(), voxelConfig());
  const TerrainPoint sensor_origin{0.0, 0.0, 0.43};
  map.update(
    {{1.01, 0.01, 0.790}, {1.02, 0.02, 0.795}, {1.03, 0.03, 0.799}},
    sensor_origin, 1.0, 0.0, true);

  const auto snapshot = map.snapshot(
    builtin_interfaces::msg::Time{}, "world", 0.0, true);
  ASSERT_EQ(snapshot.confirmed_voxel_centers.size(), 1U);
  EXPECT_NEAR(snapshot.confirmed_voxel_centers.front().z, 0.79, 1.0e-12);
  EXPECT_LE(snapshot.confirmed_voxel_centers.front().z, voxelConfig().max_height);
}

}  // namespace utree_dog_navigation
