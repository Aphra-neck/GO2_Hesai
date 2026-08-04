#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "utree_dog_msgs/msg/terrain_grid.hpp"

namespace utree_dog_navigation
{

struct TerrainMapConfig
{
  double resolution{0.05};
  double size_x{40.0};
  double size_y{40.0};
  double origin_x{-20.0};
  double origin_y{-20.0};
  int min_points_per_cell{3};
  double max_slope{0.65};
  double max_roughness{0.08};
  double max_step_height{0.24};
  double obstacle_height{0.18};
  double integration_window{1.5};
  int min_observed_frames{4};
  double height_bin_resolution{0.015};
  double confidence_frames{8.0};
};

struct TerrainPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Accumulates world-frame points and derives the terrain layers used by planning.
// This class deliberately has no ROS subscriptions so the filter can be unit tested.
class TerrainMapBuilder
{
public:
  static constexpr float kUnknown = -1000.0F;

  explicit TerrainMapBuilder(TerrainMapConfig config);

  bool addPoint(double x, double y, double z);
  void integrateFrame(const std::vector<TerrainPoint> & points, double stamp_seconds);
  utree_dog_msgs::msg::TerrainGrid build(
    const builtin_interfaces::msg::Time & stamp, const std::string & frame_id) const;

  std::size_t width() const noexcept;
  std::size_t height() const noexcept;
  const TerrainMapConfig & config() const noexcept;

private:
  struct CellStatistics
  {
    std::uint32_t count{0};
    double mean{0.0};
    double m2{0.0};
    double min_z{std::numeric_limits<double>::infinity()};
    double max_z{-std::numeric_limits<double>::infinity()};
    double stamp_seconds{0.0};

    void add(double z);
  };

  struct CellHistory
  {
    std::deque<CellStatistics> frames;
  };

  bool toGrid(double x, double y, std::size_t & gx, std::size_t & gy) const;
  std::size_t address(std::size_t x, std::size_t y) const noexcept;
  void fillIsolatedHoles(
    std::vector<float> & elevation, std::vector<float> & variance,
    std::vector<float> & roughness) const;
  void computeTerrainFeatures(
    const std::vector<float> & elevation, const std::vector<float> & roughness,
    std::vector<float> & slope, std::vector<float> & traversability) const;

  TerrainMapConfig config_;
  std::size_t width_{0};
  std::size_t height_{0};
  std::vector<CellHistory> cells_;
  double latest_stamp_seconds_{0.0};
};

}  // namespace utree_dog_navigation
