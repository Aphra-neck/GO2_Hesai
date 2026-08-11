#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "utree_dog_msgs/msg/terrain_grid.hpp"
#include "utree_dog_navigation/terrain_map_builder.hpp"

namespace utree_dog_navigation
{

struct FlatObstacleVoxelConfig
{
  double min_height{0.08};
  double max_height{0.80};
  double voxel_height{0.10};
  double clearance{0.10};
  std::size_t strong_hit_points{3U};
  std::size_t hit_confirmation_frames{2U};
  double hit_confirmation_window{0.35};
  std::size_t clear_confirmation_frames{2U};
  double clear_confirmation_window{0.35};
};

struct FlatObstacleUpdate
{
  bool accepted{false};
  bool epoch_reset{false};
  std::size_t endpoint_voxels{0U};
  std::size_t newly_confirmed_voxels{0U};
  std::size_t confirmed_voxels{0U};
  std::size_t cleared_voxels{0U};
};

struct FlatObstacleSnapshot
{
  utree_dog_msgs::msg::TerrainGrid terrain;
  std::vector<std::uint8_t> raw_obstacles;
  std::vector<std::uint8_t> inflated_obstacles;
  std::vector<TerrainPoint> confirmed_voxel_centers;
};

// Maintains navigation-specific 3D occupancy evidence from registered XT-16
// scans, then projects only confirmed obstacle voxels into the 2D planner map.
class FlatObstacleVoxelMap
{
public:
  FlatObstacleVoxelMap(
    TerrainMapConfig map_config, FlatObstacleVoxelConfig voxel_config);

  FlatObstacleUpdate update(
    const std::vector<TerrainPoint> & endpoints,
    const TerrainPoint & sensor_origin,
    double stamp_seconds, double ground_z, bool timing_continuous);
  FlatObstacleSnapshot snapshot(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id, double ground_z,
    bool include_confirmed_voxel_centers = false) const;
  void resetEpoch();

  std::size_t width() const noexcept;
  std::size_t height() const noexcept;
  std::size_t layers() const noexcept;
  const TerrainMapConfig & mapConfig() const noexcept;
  const FlatObstacleVoxelConfig & voxelConfig() const noexcept;

private:
  bool toVoxel(
    const TerrainPoint & point, double ground_z,
    std::size_t & x, std::size_t & y, std::size_t & z) const;
  void traceFreeVoxels(
    const TerrainPoint & sensor_origin, const TerrainPoint & endpoint,
    double ground_z, std::size_t endpoint_index);
  std::size_t voxelAddress(
    std::size_t x, std::size_t y, std::size_t z) const noexcept;
  std::size_t columnAddress(std::size_t x, std::size_t y) const noexcept;
  std::vector<std::uint8_t> projectRawObstacles() const;
  std::vector<std::uint8_t> inflate(
    const std::vector<std::uint8_t> & raw) const;
  void clearTransientEvidence();
  void clearEpoch();

  TerrainMapConfig map_config_;
  FlatObstacleVoxelConfig voxel_config_;
  std::size_t width_{0U};
  std::size_t height_{0U};
  std::size_t layers_{0U};
  std::vector<std::uint8_t> occupied_;
  std::vector<std::uint8_t> candidate_frames_;
  std::vector<std::uint8_t> clear_frames_;
  std::vector<std::uint16_t> frame_hits_;
  std::vector<std::uint8_t> frame_free_;
  std::vector<double> candidate_stamp_;
  std::vector<double> clear_stamp_;
  double latest_stamp_seconds_{0.0};
  bool have_stamp_{false};
  std::size_t confirmed_voxel_count_{0U};
};

}  // namespace utree_dog_navigation
