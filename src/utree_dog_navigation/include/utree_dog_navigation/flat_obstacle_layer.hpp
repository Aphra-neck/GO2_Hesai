#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "utree_dog_navigation/terrain_map_builder.hpp"

namespace utree_dog_navigation
{

struct FlatObstacleGroundFitConfig
{
  double max_range{3.0};
  double seed_height_tolerance{0.20};
  double cell_size{0.20};
  std::size_t min_points{24U};
  double min_span{0.80};
  double inlier_distance{0.04};
  double min_inlier_ratio{0.55};
  double max_rmse{0.04};
  double max_tilt{0.20};
  double max_anchor_error{0.06};
  double max_body_clearance_change{0.02};
};

struct FlatObstacleLayerConfig
{
  double resolution{0.20};
  double voxel_resolution_z{0.10};
  double size_x{40.0};
  double size_y{40.0};
  double origin_x{-20.0};
  double origin_y{-20.0};
  double min_range{0.35};
  double max_range{12.0};
  double min_height{0.08};
  double max_height{0.80};
  double self_length{0.90};
  double self_width{0.55};
  double self_height{0.70};
  double nominal_body_height{0.34};
  double obstacle_clearance{0.00};
  std::size_t hit_confirmation_frames{2U};
  double hit_confirmation_window{0.35};
  std::size_t clear_confirmation_frames{2U};
  double clear_confirmation_window{0.35};
  FlatObstacleGroundFitConfig ground_fit;
};

struct FlatObstacleFrame
{
  std::vector<TerrainPoint> points;
  TerrainPoint body_position;
  double body_yaw{0.0};
  TerrainPoint sensor_origin;
  double stamp_seconds{0.0};
  bool timing_continuous{false};
};

struct FlatGroundPlane
{
  double slope_x{0.0};
  double slope_y{0.0};
  double intercept{0.0};
  double rmse{0.0};
  std::size_t candidate_points{0U};
  std::size_t inlier_points{0U};

  double heightAt(double x, double y) const noexcept;
};

enum class FlatObstacleLayerStatus
{
  kUninitialized,
  kReady,
  kWarmingUp,
  kInvalidInput,
  kDuplicateTimestamp,
  kGroundFitFailed,
};

const char * toString(FlatObstacleLayerStatus status) noexcept;

struct FlatObstacleLayerUpdate
{
  bool accepted{false};
  bool usable{false};
  bool epoch_reset{false};
  bool reused_trusted_ground_plane{false};
  FlatObstacleLayerStatus status{FlatObstacleLayerStatus::kUninitialized};
  std::string reason;
  std::string rejected_ground_fit_reason;
  std::size_t filtered_voxels{0U};
  std::size_t newly_confirmed_voxels{0U};
  std::size_t confirmed_voxels{0U};
  std::size_t cleared_voxels{0U};
  FlatGroundPlane ground_plane;
};

struct FlatObstacleLayerSnapshot
{
  bool usable{false};
  FlatObstacleLayerStatus status{FlatObstacleLayerStatus::kUninitialized};
  std::string reason;
  double stamp_seconds{0.0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::size_t width{0U};
  std::size_t height{0U};
  FlatGroundPlane ground_plane;
  std::vector<std::uint8_t> raw_obstacles;
  std::vector<std::uint8_t> inflated_obstacles;
  std::vector<TerrainPoint> filtered_points;
  std::vector<TerrainPoint> obstacle_points;
};

// Pure flat-ground perception module. The ROS adapter supplies one exact-stamp
// world cloud/body pose pair and publishes the resulting immutable snapshot.
class FlatObstacleLayer
{
public:
  explicit FlatObstacleLayer(FlatObstacleLayerConfig config);

  FlatObstacleLayerUpdate update(const FlatObstacleFrame & frame);
  FlatObstacleLayerSnapshot snapshot() const;
  void resetEpoch();

  const FlatObstacleLayerConfig & config() const noexcept;

private:
  struct VoxelKey
  {
    std::int64_t x{0};
    std::int64_t y{0};
    std::int64_t z{0};

    bool operator==(const VoxelKey & other) const noexcept
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const noexcept;
  };

  struct VoxelEvidence
  {
    bool occupied{false};
    std::size_t candidate_frames{0U};
    double candidate_stamp{0.0};
    std::size_t clear_frames{0U};
    double clear_stamp{0.0};
    TerrainPoint representative_point;
  };

  bool fitGroundPlane(
    const FlatObstacleFrame & frame, double cos_yaw, double sin_yaw,
    FlatGroundPlane & plane,
    std::string & reason) const;
  bool trustedGroundPlaneSupported(
    const FlatObstacleFrame & frame, double cos_yaw, double sin_yaw,
    FlatGroundPlane & plane) const;
  bool pointPassesCommonFilters(
    const TerrainPoint & point, const FlatObstacleFrame & frame,
    double cos_yaw, double sin_yaw) const noexcept;
  VoxelKey voxelKey(
    double x, double y, double world_z) const noexcept;
  void traceFreeVoxels(
    const TerrainPoint & sensor_origin, const TerrainPoint & endpoint,
    const FlatGroundPlane & plane,
    std::unordered_set<VoxelKey, VoxelKeyHash> & free_voxels) const;
  std::size_t cellAddress(const TerrainPoint & point) const noexcept;
  std::vector<std::uint8_t> inflate(const std::vector<std::uint8_t> & raw) const;
  void clearState(FlatObstacleLayerStatus status, std::string reason);

  FlatObstacleLayerConfig config_;
  std::size_t width_{0U};
  std::size_t height_{0U};
  std::unordered_map<VoxelKey, VoxelEvidence, VoxelKeyHash> evidence_;
  FlatObstacleLayerSnapshot snapshot_;
  FlatGroundPlane trusted_ground_plane_;
  double latest_stamp_seconds_{0.0};
  double trusted_body_ground_clearance_{0.0};
  bool have_stamp_{false};
  bool have_trusted_ground_plane_{false};
  bool have_trusted_body_ground_clearance_{false};
  std::size_t accepted_epoch_frames_{0U};
};

}  // namespace utree_dog_navigation
