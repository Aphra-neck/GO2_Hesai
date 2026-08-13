#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "utree_dog_navigation/flat_obstacle_layer.hpp"

namespace utree_dog_navigation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kBodyHeight = 0.34;
constexpr double kSensorHeight = 0.4308;

FlatObstacleLayerConfig layerConfig(double size = 4.0)
{
  FlatObstacleLayerConfig config;
  config.resolution = 0.20;
  config.size_x = size;
  config.size_y = size;
  config.origin_x = -0.5 * size;
  config.origin_y = -0.5 * size;
  config.obstacle_clearance = 0.0;
  config.ground_fit.min_points = 24U;
  return config;
}

std::vector<TerrainPoint> groundGrid(
  double center_x = 0.0, double center_y = 0.0,
  double slope_x = 0.0, double slope_y = 0.0,
  int half_steps = 8, double step = 0.20)
{
  std::vector<TerrainPoint> result;
  const int width = 2 * half_steps + 1;
  result.reserve(static_cast<std::size_t>(width * width));
  for (int x_index = -half_steps; x_index <= half_steps; ++x_index) {
    const double x = center_x + step * static_cast<double>(x_index);
    for (int y_index = -half_steps; y_index <= half_steps; ++y_index) {
      const double y = center_y + step * static_cast<double>(y_index);
      result.push_back({x, y, slope_x * x + slope_y * y});
    }
  }
  return result;
}

std::vector<TerrainPoint> negativeXGround()
{
  std::vector<TerrainPoint> result;
  for (int x_index = -16; x_index <= -4; ++x_index) {
    for (int y_index = -10; y_index <= 10; ++y_index) {
      result.push_back({0.1 * x_index, 0.1 * y_index, 0.0});
    }
  }
  return result;
}

FlatObstacleFrame frame(
  std::vector<TerrainPoint> points, double stamp, bool continuous,
  double body_x = 0.0, double body_y = 0.0, double ground_z = 0.0)
{
  return {
    std::move(points),
    {body_x, body_y, ground_z + kBodyHeight},
    0.0,
    {body_x, body_y, ground_z + kSensorHeight},
    stamp,
    continuous};
}

std::size_t countOccupied(const std::vector<std::uint8_t> & mask)
{
  return static_cast<std::size_t>(std::count(
           mask.begin(), mask.end(), static_cast<std::uint8_t>(1U)));
}

std::size_t countRaw(const FlatObstacleLayerSnapshot & snapshot)
{
  return countOccupied(snapshot.raw_obstacles);
}
}  // namespace

TEST(FlatObstacleLayer, RejectsSingleFrameObstacleConfirmation)
{
  auto config = layerConfig();
  config.hit_confirmation_frames = 1U;

  EXPECT_THROW(FlatObstacleLayer layer(config), std::invalid_argument);
}

TEST(FlatObstacleLayer, RejectsInvalidVerticalVoxelResolution)
{
  auto config = layerConfig();
  config.voxel_resolution_z = 0.0;
  EXPECT_THROW(FlatObstacleLayer layer(config), std::invalid_argument);

  config.voxel_resolution_z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(FlatObstacleLayer layer(config), std::invalid_argument);
}

TEST(FlatObstacleLayer, RejectsGroundAnchorErrorAtObstacleHeightThreshold)
{
  auto config = layerConfig();
  config.ground_fit.max_anchor_error = config.min_height;

  EXPECT_THROW(FlatObstacleLayer layer(config), std::invalid_argument);
}

TEST(FlatObstacleLayer, TiltedFlatWorldPlaneRemainsFreeAcrossMovingBodyPoses)
{
  auto config = layerConfig();
  const double slope = std::tan(2.61 * kPi / 180.0);
  FlatObstacleLayer layer(config);
  const auto floor = groundGrid(0.0, 0.0, slope, 0.0, 9, 0.20);

  const double first_x = -0.2;
  const auto first = layer.update(
    frame(
      floor, 1.0, false, first_x, 0.0, slope * first_x));
  ASSERT_TRUE(first.accepted);
  EXPECT_FALSE(first.usable);
  EXPECT_EQ(first.confirmed_voxels, 0U);

  const double second_x = 0.2;
  const auto second = layer.update(
    frame(
      floor, 1.1, true, second_x, 0.0, slope * second_x));
  const auto snapshot = layer.snapshot();

  ASSERT_TRUE(second.usable) << second.reason;
  EXPECT_NEAR(second.ground_plane.slope_x, slope, 1.0e-6);
  EXPECT_NEAR(second.ground_plane.slope_y, 0.0, 1.0e-6);
  EXPECT_EQ(second.confirmed_voxels, 0U);
  EXPECT_EQ(countRaw(snapshot), 0U);
  EXPECT_TRUE(snapshot.obstacle_points.empty());
}

TEST(FlatObstacleLayer, DensePointsRequireDistinctSourceTimestampsToConfirm)
{
  FlatObstacleLayer layer(layerConfig());
  auto points = groundGrid();
  points.insert(
    points.end(), {
      {1.01, 0.01, 0.25}, {1.02, 0.02, 0.26}, {1.03, 0.03, 0.27}});

  const auto first = layer.update(frame(points, 1.0, false));
  EXPECT_EQ(first.newly_confirmed_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);

  const auto duplicate = layer.update(frame(points, 1.0, true));
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_EQ(duplicate.status, FlatObstacleLayerStatus::kDuplicateTimestamp);
  EXPECT_EQ(duplicate.confirmed_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);

  const auto second = layer.update(frame(points, 1.1, true));
  ASSERT_TRUE(second.usable) << second.reason;
  EXPECT_EQ(second.newly_confirmed_voxels, 1U);
  EXPECT_EQ(second.confirmed_voxels, 1U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
}

TEST(FlatObstacleLayer, AlternatingHeightsInOneColumnDoNotConfirmEachOther)
{
  FlatObstacleLayer layer(layerConfig());
  auto low_return = negativeXGround();
  low_return.push_back({1.01, 0.01, 0.25});
  auto high_return = negativeXGround();
  high_return.push_back({1.01, 0.01, 0.55});

  layer.update(frame(low_return, 1.0, false));
  const auto second = layer.update(frame(high_return, 1.1, true));

  EXPECT_EQ(second.newly_confirmed_voxels, 0U);
  EXPECT_EQ(second.confirmed_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);
  EXPECT_TRUE(layer.snapshot().obstacle_points.empty());
}

TEST(FlatObstacleLayer, GroundPlaneShiftCannotMakeDifferentWorldHeightsConfirm)
{
  FlatObstacleLayer layer(layerConfig());
  auto first_points = negativeXGround();
  first_points.push_back({1.01, 0.01, 0.25});
  auto shifted_points = negativeXGround();
  for (auto & point : shifted_points) {
    point.z += 0.20;
  }
  shifted_points.push_back({1.01, 0.01, 0.45});

  layer.update(frame(first_points, 1.0, false));
  const auto shifted = layer.update(frame(
      shifted_points, 1.1, true, 0.0, 0.0, 0.20));

  EXPECT_EQ(shifted.newly_confirmed_voxels, 0U);
  EXPECT_EQ(shifted.confirmed_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);
  EXPECT_TRUE(layer.snapshot().obstacle_points.empty());
}

TEST(FlatObstacleLayer, LowRayDoesNotClearHigherVoxelInSameColumn)
{
  auto config = layerConfig();
  FlatObstacleLayer layer(config);
  auto with_high_obstacle = negativeXGround();
  with_high_obstacle.push_back({1.01, 0.01, 0.65});
  layer.update(frame(with_high_obstacle, 1.0, false));
  layer.update(frame(with_high_obstacle, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  auto low_ray = negativeXGround();
  low_ray.push_back({1.80, 0.0, 0.0});
  layer.update(frame(low_ray, 1.2, true));
  const auto second_clear = layer.update(frame(low_ray, 1.3, true));

  EXPECT_EQ(second_clear.cleared_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
  ASSERT_EQ(layer.snapshot().obstacle_points.size(), 1U);
  EXPECT_DOUBLE_EQ(layer.snapshot().obstacle_points.front().z, 0.65);
}

TEST(FlatObstacleLayer, GroundPlaneShiftCannotMakeDifferentWorldHeightRayClear)
{
  auto config = layerConfig();
  FlatObstacleLayer layer(config);
  auto with_obstacle = negativeXGround();
  with_obstacle.push_back({1.01, 0.01, 0.25});
  layer.update(frame(with_obstacle, 1.0, false));
  layer.update(frame(with_obstacle, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  auto shifted_ray = negativeXGround();
  for (auto & point : shifted_ray) {
    point.z += 0.20;
  }
  shifted_ray.push_back({1.80, 0.0, 0.25});
  layer.update(frame(shifted_ray, 1.2, true, 0.0, 0.0, 0.20));
  const auto second_ray = layer.update(frame(
      shifted_ray, 1.3, true, 0.0, 0.0, 0.20));

  EXPECT_EQ(second_ray.cleared_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
  ASSERT_EQ(layer.snapshot().obstacle_points.size(), 1U);
  EXPECT_DOUBLE_EQ(layer.snapshot().obstacle_points.front().z, 0.25);
}

TEST(FlatObstacleLayer, RayThroughSameHeightVoxelClearsAfterTwoFrames)
{
  auto config = layerConfig();
  FlatObstacleLayer layer(config);
  auto with_obstacle = negativeXGround();
  with_obstacle.push_back({1.01, 0.01, 0.25});
  layer.update(frame(with_obstacle, 1.0, false));
  layer.update(frame(with_obstacle, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  auto same_height_ray = negativeXGround();
  same_height_ray.push_back({1.80, 0.0, 0.05});
  const auto first_clear = layer.update(frame(same_height_ray, 1.2, true));
  EXPECT_EQ(first_clear.cleared_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);

  const auto second_clear = layer.update(frame(same_height_ray, 1.3, true));
  EXPECT_EQ(second_clear.cleared_voxels, 1U);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);
}

TEST(FlatObstacleLayer, MultipleHeightVoxelsProjectToOneRawCell)
{
  FlatObstacleLayer layer(layerConfig());
  auto points = negativeXGround();
  points.push_back({1.01, 0.01, 0.25});
  points.push_back({1.01, 0.01, 0.55});

  layer.update(frame(points, 1.0, false));
  const auto confirmed = layer.update(frame(points, 1.1, true));
  const auto snapshot = layer.snapshot();

  EXPECT_EQ(confirmed.newly_confirmed_voxels, 2U);
  EXPECT_EQ(confirmed.confirmed_voxels, 2U);
  EXPECT_EQ(countRaw(snapshot), 1U);
  ASSERT_EQ(snapshot.obstacle_points.size(), 2U);
  EXPECT_TRUE(
    std::any_of(
      snapshot.obstacle_points.begin(), snapshot.obstacle_points.end(),
      [](const TerrainPoint & point) {return point.z == 0.25;}));
  EXPECT_TRUE(
    std::any_of(
      snapshot.obstacle_points.begin(), snapshot.obstacle_points.end(),
      [](const TerrainPoint & point) {return point.z == 0.55;}));
}

TEST(FlatObstacleLayer, FreeRayBetweenHitsCancelsUnconfirmedCandidate)
{
  auto config = layerConfig();
  FlatObstacleLayer layer(config);
  const auto ground = negativeXGround();
  auto with_column = ground;
  with_column.push_back({1.01, 0.01, 0.25});
  auto free_ray = ground;
  free_ray.push_back({1.80, 0.0, 0.05});

  layer.update(frame(with_column, 1.0, false));
  layer.update(frame(free_ray, 1.1, true));
  const auto hit_after_free = layer.update(frame(with_column, 1.2, true));

  EXPECT_EQ(hit_after_free.newly_confirmed_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);

  const auto next_hit = layer.update(frame(with_column, 1.3, true));
  EXPECT_EQ(next_hit.newly_confirmed_voxels, 1U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
}

TEST(FlatObstacleLayer, ConfirmedVoxelPersistsWithoutExactFreeRay)
{
  FlatObstacleLayer layer(layerConfig());
  const auto ground = negativeXGround();
  auto with_column = ground;
  with_column.push_back({1.01, 0.01, 0.25});
  layer.update(frame(with_column, 1.0, false));
  const auto confirmed = layer.update(frame(with_column, 1.1, true));
  ASSERT_EQ(confirmed.newly_confirmed_voxels, 1U);
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  FlatObstacleLayerUpdate latest;
  for (int step = 1; step <= 12; ++step) {
    latest = layer.update(frame(
        ground, 1.1 + 0.1 * static_cast<double>(step), true));
    EXPECT_FALSE(latest.epoch_reset);
  }

  EXPECT_EQ(latest.cleared_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
  ASSERT_EQ(layer.snapshot().obstacle_points.size(), 1U);
  EXPECT_DOUBLE_EQ(layer.snapshot().obstacle_points.front().z, 0.25);
}

TEST(FlatObstacleLayer, TwoDistinctFreeRayFramesClearConfirmedVoxel)
{
  auto config = layerConfig();
  FlatObstacleLayer layer(config);
  const auto ground = negativeXGround();
  auto with_column = ground;
  with_column.push_back({1.01, 0.01, 0.25});
  auto free_ray = ground;
  free_ray.push_back({1.80, 0.0, 0.05});
  layer.update(frame(with_column, 1.0, false));
  layer.update(frame(with_column, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  const auto first_clear = layer.update(frame(free_ray, 1.2, true));
  EXPECT_EQ(first_clear.cleared_voxels, 0U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);

  const auto second_clear = layer.update(frame(free_ray, 1.3, true));
  EXPECT_EQ(second_clear.cleared_voxels, 1U);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);
}

TEST(FlatObstacleLayer, RayAboveConfirmedObstacleCannotClearIt)
{
  auto config = layerConfig();
  FlatObstacleLayer layer(config);
  const auto ground = negativeXGround();
  auto with_column = ground;
  with_column.push_back({1.01, 0.01, 0.25});
  layer.update(frame(with_column, 1.0, false));
  layer.update(frame(with_column, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  auto high_ray = ground;
  high_ray.push_back({1.6, 0.0, 0.90});
  layer.update(frame(high_ray, 1.2, true));
  layer.update(frame(high_ray, 1.3, true));

  const auto snapshot = layer.snapshot();
  EXPECT_TRUE(
    std::any_of(
      snapshot.obstacle_points.begin(), snapshot.obstacle_points.end(),
      [](const TerrainPoint & point) {return std::abs(point.x - 1.01) < 1.0e-12;}));
}

TEST(FlatObstacleLayer, RayBelowFittedGroundCannotClearObstacleColumn)
{
  auto config = layerConfig();
  FlatObstacleLayer layer(config);
  const auto ground = negativeXGround();
  auto with_column = ground;
  with_column.push_back({1.01, 0.01, 0.25});
  layer.update(frame(with_column, 1.0, false));
  layer.update(frame(with_column, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  auto below_ground_ray = ground;
  below_ground_ray.push_back({1.6, 0.0, -0.50});
  layer.update(frame(below_ground_ray, 1.2, true));
  layer.update(frame(below_ground_ray, 1.3, true));

  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
}

TEST(FlatObstacleLayer, ObstacleEndpointWinsOverFreeRayInSameFrame)
{
  auto config = layerConfig();
  FlatObstacleLayer layer(config);
  auto points = groundGrid();
  points.push_back({1.01, 0.01, 0.25});
  points.push_back({1.8, 0.0, 0.0});
  layer.update(frame(points, 1.0, false));
  layer.update(frame(points, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  layer.update(frame(points, 1.2, true));
  layer.update(frame(points, 1.3, true));

  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
}

TEST(FlatObstacleLayer, RollingWindowEvictsHistoryWithoutResettingEpoch)
{
  auto config = layerConfig(2.0);
  config.clear_confirmation_frames = 100U;
  FlatObstacleLayer layer(config);
  auto initial = groundGrid(0.0, 0.0, 0.0, 0.0, 8, 0.10);
  initial.push_back({0.61, 0.01, 0.25});
  layer.update(frame(initial, 1.0, false));
  layer.update(frame(initial, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  FlatObstacleLayerUpdate latest;
  for (int step = 1; step <= 5; ++step) {
    const double x = 0.4 * static_cast<double>(step);
    latest = layer.update(
      frame(
        groundGrid(x, 0.0, 0.0, 0.0, 8, 0.10),
        1.1 + 0.1 * static_cast<double>(step), true, x));
    EXPECT_FALSE(latest.epoch_reset);
  }
  const auto snapshot = layer.snapshot();

  EXPECT_NEAR(snapshot.origin_x, 1.0, 1.0e-12);
  EXPECT_EQ(countRaw(snapshot), 0U);
  EXPECT_TRUE(snapshot.obstacle_points.empty());
}

TEST(FlatObstacleLayer, GroundFitUsesFloorUnderDenseElevatedReturns)
{
  FlatObstacleLayer layer(layerConfig());
  const double slope = std::tan(2.61 * kPi / 180.0);
  std::vector<TerrainPoint> points;
  for (int x_index = -8; x_index <= 8; ++x_index) {
    const double x = 0.2 * static_cast<double>(x_index);
    for (int y_index = -8; y_index <= 8; ++y_index) {
      const double y = 0.2 * static_cast<double>(y_index);
      const double floor_z = slope * x;
      points.push_back({x, y, floor_z});
      if (x < -0.4) {
        points.push_back({x, y, floor_z + 0.12});
      }
    }
  }
  const auto update = layer.update(frame(points, 1.0, false));

  ASSERT_TRUE(update.accepted);
  ASSERT_NE(update.status, FlatObstacleLayerStatus::kGroundFitFailed) << update.reason;
  EXPECT_NEAR(update.ground_plane.slope_x, slope, 0.005);
  EXPECT_NEAR(update.ground_plane.slope_y, 0.0, 0.005);
  EXPECT_NEAR(update.ground_plane.intercept, 0.0, 0.01);
}

TEST(FlatObstacleLayer, GroundFitUsesDominantSurfaceWhenSecondaryReturnsExceedInlierDistance)
{
  FlatObstacleLayer layer(layerConfig());
  std::vector<TerrainPoint> points;
  for (int x_index = -12; x_index <= 11; ++x_index) {
    const double x = 0.2 * (static_cast<double>(x_index) + 0.5) + 0.001;
    for (int y_index = -4; y_index <= 3; ++y_index) {
      const double y = 0.2 * (static_cast<double>(y_index) + 0.5) + 0.001;
      const int raw_code = (3 * x_index + 7 * y_index) % 10;
      const int code = (raw_code + 10) % 10;
      const double z = code < 6 ? 0.0 : (code < 9 ? 0.10 : -0.08);
      points.push_back({x, y, z});
    }
  }

  const auto update = layer.update(frame(points, 1.0, false));

  ASSERT_TRUE(update.accepted);
  ASSERT_NE(update.status, FlatObstacleLayerStatus::kGroundFitFailed) << update.reason;
  EXPECT_EQ(update.ground_plane.candidate_points, 180U);
  EXPECT_EQ(update.ground_plane.inlier_points, 108U);
  EXPECT_NEAR(update.ground_plane.slope_x, 0.0, 1.0e-6);
  EXPECT_NEAR(update.ground_plane.slope_y, 0.0, 1.0e-6);
  EXPECT_NEAR(update.ground_plane.intercept, 0.0, 1.0e-6);
  EXPECT_LE(update.ground_plane.rmse, layerConfig().ground_fit.max_rmse);
  EXPECT_GE(
    static_cast<double>(update.ground_plane.inlier_points) /
    static_cast<double>(update.ground_plane.candidate_points),
    layerConfig().ground_fit.min_inlier_ratio);
}

TEST(FlatObstacleLayer, GroundFitRejectsInliersWithoutTwoDimensionalCoverage)
{
  FlatObstacleLayer layer(layerConfig());
  std::vector<TerrainPoint> points;
  for (int x_index = -8; x_index <= 7; ++x_index) {
    const double x = 0.2 * (static_cast<double>(x_index) + 0.5) + 0.001;
    for (int y_index = -1; y_index <= 1; ++y_index) {
      const double y = 0.2 * (static_cast<double>(y_index) + 0.5) + 0.001;
      points.push_back({x, y, 0.0});
    }
  }
  for (int x_index = -5; x_index <= 5; x_index += 2) {
    const double x = 0.2 * (static_cast<double>(x_index) + 0.5) + 0.001;
    points.push_back({x, -1.099, 0.12});
    points.push_back({x, 1.101, 0.12});
  }

  const auto update = layer.update(frame(points, 1.0, false));

  EXPECT_TRUE(update.accepted);
  EXPECT_FALSE(update.usable);
  EXPECT_EQ(update.status, FlatObstacleLayerStatus::kGroundFitFailed);
  EXPECT_EQ(update.reason, "insufficient_ground_inlier_span");
  EXPECT_GE(
    static_cast<double>(update.ground_plane.inlier_points) /
    static_cast<double>(update.ground_plane.candidate_points),
    layerConfig().ground_fit.min_inlier_ratio);
}

TEST(FlatObstacleLayer, ElevatedSurfaceCannotReplaceExpectedStandingGround)
{
  FlatObstacleLayer layer(layerConfig());
  auto elevated_surface = groundGrid();
  for (auto & point : elevated_surface) {
    point.z += 0.12;
  }

  const auto update = layer.update(frame(elevated_surface, 1.0, false));

  EXPECT_TRUE(update.accepted);
  EXPECT_FALSE(update.usable);
  EXPECT_EQ(update.status, FlatObstacleLayerStatus::kGroundFitFailed);
  EXPECT_EQ(update.reason, "ground_anchor_error_above_limit");
  EXPECT_FALSE(layer.snapshot().usable);
}

TEST(FlatObstacleLayer, GroundFitFailurePublishesUnusableNotFreeSnapshot)
{
  FlatObstacleLayer layer(layerConfig());
  const auto ground = groundGrid(0.0, 0.0, 0.05, 0.0);
  layer.update(frame(ground, 1.0, false));
  layer.update(frame(ground, 1.1, true));
  ASSERT_TRUE(layer.snapshot().usable);
  ASSERT_NEAR(layer.snapshot().ground_plane.slope_x, 0.05, 1.0e-6);

  const auto failed = layer.update(frame({{1.0, 0.0, 0.0}}, 1.2, true));
  const auto snapshot = layer.snapshot();

  EXPECT_TRUE(failed.accepted);
  EXPECT_FALSE(failed.usable);
  EXPECT_EQ(failed.status, FlatObstacleLayerStatus::kGroundFitFailed);
  EXPECT_EQ(failed.reason, "insufficient_ground_candidates");
  EXPECT_FALSE(snapshot.usable);
  EXPECT_EQ(snapshot.status, FlatObstacleLayerStatus::kGroundFitFailed);
  EXPECT_DOUBLE_EQ(snapshot.stamp_seconds, 1.2);
  EXPECT_DOUBLE_EQ(snapshot.ground_plane.slope_x, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.ground_plane.slope_y, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.ground_plane.intercept, 0.0);
  EXPECT_EQ(countRaw(snapshot), 0U);
}

TEST(FlatObstacleLayer, BackwardTimestampResetsEpochAndNeedsTwoNewFrames)
{
  auto config = layerConfig();
  config.clear_confirmation_frames = 100U;
  FlatObstacleLayer layer(config);
  auto old_points = groundGrid();
  old_points.push_back({0.61, 0.01, 0.25});
  layer.update(frame(old_points, 2.0, false));
  layer.update(frame(old_points, 2.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  auto new_points = groundGrid();
  new_points.push_back({-0.61, 0.01, 0.25});
  const auto reset = layer.update(frame(new_points, 1.0, true));
  EXPECT_TRUE(reset.epoch_reset);
  EXPECT_FALSE(reset.usable);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);

  const auto ready = layer.update(frame(new_points, 1.1, true));
  ASSERT_TRUE(ready.usable);
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);
  ASSERT_EQ(layer.snapshot().obstacle_points.size(), 1U);
  EXPECT_LT(layer.snapshot().obstacle_points.front().x, 0.0);
}

TEST(FlatObstacleLayer, TimingDiscontinuityResetsEpochAndNeedsTwoNewFrames)
{
  FlatObstacleLayer layer(layerConfig());
  auto old_points = groundGrid();
  old_points.push_back({0.61, 0.01, 0.25});
  layer.update(frame(old_points, 1.0, false));
  layer.update(frame(old_points, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  auto new_points = groundGrid();
  new_points.push_back({-0.61, 0.01, 0.25});
  const auto reset = layer.update(frame(new_points, 2.0, false));
  EXPECT_TRUE(reset.epoch_reset);
  EXPECT_FALSE(reset.usable);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);

  const auto ready = layer.update(frame(new_points, 2.1, true));
  EXPECT_TRUE(ready.usable);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
}

TEST(FlatObstacleLayer, LargeForwardTimestampGapResetsEpoch)
{
  FlatObstacleLayer layer(layerConfig());
  auto old_points = groundGrid();
  old_points.push_back({0.61, 0.01, 0.25});
  layer.update(frame(old_points, 1.0, false));
  layer.update(frame(old_points, 1.1, true));
  ASSERT_EQ(countRaw(layer.snapshot()), 1U);

  auto new_points = groundGrid();
  new_points.push_back({-0.61, 0.01, 0.25});
  const auto reset = layer.update(frame(new_points, 2.0, true));

  EXPECT_TRUE(reset.epoch_reset);
  EXPECT_FALSE(reset.usable);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);

  const auto ready = layer.update(frame(new_points, 2.1, true));
  EXPECT_TRUE(ready.usable);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
}

TEST(FlatObstacleLayer, InflationNeverChangesRawOccupancy)
{
  auto config = layerConfig();
  config.obstacle_clearance = 0.10;
  FlatObstacleLayer layer(config);
  auto points = groundGrid();
  points.push_back({0.61, 0.01, 0.25});
  layer.update(frame(points, 1.0, false));
  layer.update(frame(points, 1.1, true));
  const auto snapshot = layer.snapshot();

  EXPECT_EQ(countRaw(snapshot), 1U);
  EXPECT_EQ(countOccupied(snapshot.inflated_obstacles), 9U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
}

TEST(FlatObstacleLayer, SnapshotContainsActualFilteredSourcePoint)
{
  FlatObstacleLayer layer(layerConfig());
  auto points = groundGrid();
  const TerrainPoint obstacle{0.67, 0.13, 0.31};
  points.push_back(obstacle);
  layer.update(frame(points, 1.0, false));
  layer.update(frame(points, 1.1, true));
  const auto snapshot = layer.snapshot();

  ASSERT_EQ(snapshot.obstacle_points.size(), 1U);
  EXPECT_DOUBLE_EQ(snapshot.obstacle_points.front().x, obstacle.x);
  EXPECT_DOUBLE_EQ(snapshot.obstacle_points.front().y, obstacle.y);
  EXPECT_DOUBLE_EQ(snapshot.obstacle_points.front().z, obstacle.z);
}

TEST(FlatObstacleLayer, SnapshotSeparatesCurrentFilteredAndConfirmedPoints)
{
  auto config = layerConfig();
  config.clear_confirmation_frames = 100U;
  FlatObstacleLayer layer(config);
  auto points = negativeXGround();
  const TerrainPoint obstacle{0.67, 0.13, 0.31};
  points.push_back(obstacle);

  layer.update(frame(points, 1.0, false));
  auto snapshot = layer.snapshot();
  ASSERT_EQ(snapshot.filtered_points.size(), 1U);
  EXPECT_DOUBLE_EQ(snapshot.filtered_points.front().x, obstacle.x);
  EXPECT_TRUE(snapshot.obstacle_points.empty());

  layer.update(frame(points, 1.1, true));
  snapshot = layer.snapshot();
  ASSERT_EQ(snapshot.filtered_points.size(), 1U);
  ASSERT_EQ(snapshot.obstacle_points.size(), 1U);
  EXPECT_DOUBLE_EQ(snapshot.filtered_points.front().z, obstacle.z);
  EXPECT_DOUBLE_EQ(snapshot.obstacle_points.front().z, obstacle.z);

  layer.update(frame(negativeXGround(), 1.2, true));
  snapshot = layer.snapshot();
  EXPECT_TRUE(snapshot.filtered_points.empty());
  ASSERT_EQ(snapshot.obstacle_points.size(), 1U);
  EXPECT_DOUBLE_EQ(snapshot.obstacle_points.front().z, obstacle.z);
}

TEST(FlatObstacleLayer, FailureClosedSnapshotClearsLiveAndConfirmedPoints)
{
  FlatObstacleLayer layer(layerConfig());
  auto points = negativeXGround();
  points.push_back({0.67, 0.13, 0.31});
  layer.update(frame(points, 1.0, false));
  layer.update(frame(points, 1.1, true));
  ASSERT_FALSE(layer.snapshot().filtered_points.empty());
  ASSERT_FALSE(layer.snapshot().obstacle_points.empty());

  const auto failed = layer.update(frame({{1.0, 0.0, 0.0}}, 1.2, true));

  EXPECT_EQ(failed.status, FlatObstacleLayerStatus::kGroundFitFailed);
  EXPECT_TRUE(layer.snapshot().filtered_points.empty());
  EXPECT_TRUE(layer.snapshot().obstacle_points.empty());
}

TEST(FlatObstacleLayer, ResetEpochClearsLiveAndConfirmedPoints)
{
  FlatObstacleLayer layer(layerConfig());
  auto points = negativeXGround();
  points.push_back({0.67, 0.13, 0.31});
  layer.update(frame(points, 1.0, false));
  layer.update(frame(points, 1.1, true));
  ASSERT_FALSE(layer.snapshot().filtered_points.empty());
  ASSERT_FALSE(layer.snapshot().obstacle_points.empty());

  layer.resetEpoch();

  EXPECT_FALSE(layer.snapshot().usable);
  EXPECT_EQ(layer.snapshot().status, FlatObstacleLayerStatus::kUninitialized);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);
  EXPECT_TRUE(layer.snapshot().filtered_points.empty());
  EXPECT_TRUE(layer.snapshot().obstacle_points.empty());
}

TEST(FlatObstacleLayer, RangeSelfAndCollisionHeightFiltersAreApplied)
{
  FlatObstacleLayer layer(layerConfig());
  auto points = groundGrid();
  points.insert(
    points.end(), {
      {0.20, 0.0, 0.25},
      {13.0, 0.0, 0.25},
      {1.20, 0.0, 0.90},
      {0.80, 0.10, 0.25}});
  layer.update(frame(points, 1.0, false));
  const auto ready = layer.update(frame(points, 1.1, true));

  EXPECT_EQ(ready.confirmed_voxels, 1U);
  EXPECT_EQ(countRaw(layer.snapshot()), 1U);
  ASSERT_EQ(layer.snapshot().obstacle_points.size(), 1U);
  EXPECT_DOUBLE_EQ(layer.snapshot().obstacle_points.front().x, 0.80);
}

TEST(FlatObstacleLayer, NonfiniteFrameMetadataFailsClosed)
{
  FlatObstacleLayer layer(layerConfig());
  auto invalid = frame(groundGrid(), 1.0, false);
  invalid.body_position.x = std::numeric_limits<double>::quiet_NaN();

  const auto update = layer.update(invalid);

  EXPECT_FALSE(update.accepted);
  EXPECT_FALSE(update.usable);
  EXPECT_EQ(update.status, FlatObstacleLayerStatus::kInvalidInput);
  EXPECT_FALSE(layer.snapshot().usable);
  EXPECT_EQ(countRaw(layer.snapshot()), 0U);
}

}  // namespace utree_dog_navigation
