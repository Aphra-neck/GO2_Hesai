#include "utree_dog_navigation/terrain_map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <unordered_map>

namespace utree_dog_navigation
{

TerrainMapBuilder::TerrainMapBuilder(TerrainMapConfig config)
: config_(std::move(config)),
  width_(static_cast<std::size_t>(std::ceil(config_.size_x / config_.resolution))),
  height_(static_cast<std::size_t>(std::ceil(config_.size_y / config_.resolution))),
  cells_(width_ * height_)
{
}

void TerrainMapBuilder::CellStatistics::add(double z)
{
  // Welford's update avoids the cancellation error of sum(z^2) - sum(z)^2.
  ++count;
  const double delta = z - mean;
  mean += delta / static_cast<double>(count);
  m2 += delta * (z - mean);
  min_z = std::min(min_z, z);
  max_z = std::max(max_z, z);
}

bool TerrainMapBuilder::addPoint(double x, double y, double z)
{
  std::size_t gx = 0;
  std::size_t gy = 0;
  if (!toGrid(x, y, gx, gy)) {
    return false;
  }
  // Compatibility path for unit tests and non-streaming callers: one point is one frame.
  auto & frames = cells_[address(gx, gy)].frames;
  CellStatistics observation;
  observation.stamp_seconds = latest_stamp_seconds_;
  observation.add(z);
  frames.push_back(observation);
  return true;
}

std::size_t TerrainMapBuilder::integrateFrame(
  const std::vector<TerrainPoint> & points, double stamp_seconds)
{
  latest_stamp_seconds_ = stamp_seconds;
  // Aggregate a scan once per grid cell. This prevents a dense scan from dominating
  // several independent Mid360 observations of the same tread.
  std::unordered_map<std::size_t, std::vector<double>> frame_cells;
  for (const auto & point : points) {
    std::size_t gx = 0;
    std::size_t gy = 0;
    if (!toGrid(point.x, point.y, gx, gy)) {continue;}
    frame_cells[address(gx, gy)].push_back(point.z);
  }
  const double oldest = stamp_seconds - config_.integration_window;
  for (auto & history : cells_) {
    while (!history.frames.empty() && history.frames.front().stamp_seconds < oldest) {
      history.frames.pop_front();
    }
  }
  for (auto & entry : frame_cells) {
    auto & heights = entry.second;
    std::sort(heights.begin(), heights.end());
    CellStatistics observation;
    observation.stamp_seconds = stamp_seconds;
    // One representative height per cell per scan gives every scan equal temporal
    // weight. Keep the raw span to identify cells intersecting a vertical riser.
    observation.add(heights[heights.size() / 2]);
    observation.min_z = heights.front();
    observation.max_z = heights.back();
    cells_[entry.first].frames.push_back(std::move(observation));
  }
  return frame_cells.size();
}

utree_dog_msgs::msg::TerrainGrid TerrainMapBuilder::build(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id) const
{
  const std::size_t cell_count = width_ * height_;
  std::vector<float> elevation(cell_count, kUnknown);
  std::vector<float> variance(cell_count, kUnknown);
  std::vector<float> slope(cell_count, kUnknown);
  std::vector<float> roughness(cell_count, kUnknown);
  std::vector<float> traversability(cell_count, kUnknown);
  std::vector<std::uint16_t> observation_count(cell_count, 0);
  std::vector<float> confidence(cell_count, 0.0F);
  std::vector<float> age(cell_count, kUnknown);
  std::vector<CellStatistics> fused(cell_count);

  for (std::size_t i = 0; i < cell_count; ++i) {
    const auto & history = cells_[i].frames;
    if (history.empty()) {continue;}
    auto & cell = fused[i];
    std::vector<double> frame_heights;
    frame_heights.reserve(history.size());
    for (const auto & frame : history) {
      if (frame.count > 0U) {
        frame_heights.push_back(frame.mean);
      }
    }
    observation_count[i] = static_cast<std::uint16_t>(
      std::min<std::size_t>(frame_heights.size(), std::numeric_limits<std::uint16_t>::max()));
    age[i] = static_cast<float>(std::max(0.0, latest_stamp_seconds_ - history.back().stamp_seconds));
    confidence[i] = static_cast<float>(std::clamp(
      frame_heights.size() / config_.confidence_frames, 0.0, 1.0));
    if (frame_heights.size() < static_cast<std::size_t>(config_.min_observed_frames)) {
      continue;
    }
    // Select the dominant height cluster. It rejects isolated riser/outlier returns and
    // preserves the tread surface instead of biasing elevation toward the lowest point.
    std::sort(frame_heights.begin(), frame_heights.end());
    std::size_t best_begin = 0;
    std::size_t best_end = 0;
    for (std::size_t begin = 0, end = 0; begin < frame_heights.size(); ++begin) {
      end = std::max(end, begin);
      while (end + 1 < frame_heights.size() &&
        frame_heights[end + 1] - frame_heights[begin] <= config_.height_bin_resolution) {++end;}
      if (end - begin > best_end - best_begin) {best_begin = begin; best_end = end;}
    }
    for (std::size_t j = best_begin; j <= best_end; ++j) {cell.add(frame_heights[j]);}
    elevation[i] = static_cast<float>(cell.mean);
    variance[i] = static_cast<float>(cell.count > 1 ? cell.m2 / (cell.count - 1) : 0.0);
    roughness[i] = std::sqrt(std::max(0.0F, variance[i]));
    confidence[i] *= static_cast<float>(cell.count) / static_cast<float>(frame_heights.size());
  }

  fillIsolatedHoles(elevation, variance, roughness);
  computeTerrainFeatures(elevation, roughness, slope, traversability);

  utree_dog_msgs::msg::TerrainGrid result;
  result.header.stamp = stamp;
  result.header.frame_id = frame_id;
  result.resolution = config_.resolution;
  result.width = static_cast<std::uint32_t>(width_);
  result.height = static_cast<std::uint32_t>(height_);
  result.origin_x = config_.origin_x;
  result.origin_y = config_.origin_y;
  result.unknown_value = kUnknown;
  result.elevation = std::move(elevation);
  result.variance = std::move(variance);
  result.slope = std::move(slope);
  result.roughness = std::move(roughness);
  result.traversability = std::move(traversability);
  result.observation_count = std::move(observation_count);
  result.confidence = std::move(confidence);
  result.age = std::move(age);
  return result;
}

std::size_t TerrainMapBuilder::width() const noexcept {return width_;}
std::size_t TerrainMapBuilder::height() const noexcept {return height_;}
const TerrainMapConfig & TerrainMapBuilder::config() const noexcept {return config_;}

bool TerrainMapBuilder::toGrid(double x, double y, std::size_t & gx, std::size_t & gy) const
{
  const int ix = static_cast<int>(std::floor((x - config_.origin_x) / config_.resolution));
  const int iy = static_cast<int>(std::floor((y - config_.origin_y) / config_.resolution));
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(width_) || iy >= static_cast<int>(height_)) {
    return false;
  }
  gx = static_cast<std::size_t>(ix);
  gy = static_cast<std::size_t>(iy);
  return true;
}

std::size_t TerrainMapBuilder::address(std::size_t x, std::size_t y) const noexcept
{
  return y * width_ + x;
}

void TerrainMapBuilder::fillIsolatedHoles(
  std::vector<float> & elevation, std::vector<float> & variance,
  std::vector<float> & roughness) const
{
  // Fill only a one-cell hole surrounded by measurements. Large unknown regions stay unknown.
  std::vector<float> filtered = elevation;
  for (std::size_t y = 1; y + 1 < height_; ++y) {
    for (std::size_t x = 1; x + 1 < width_; ++x) {
      const std::size_t index = address(x, y);
      if (elevation[index] != kUnknown) {
        continue;
      }
      std::vector<float> neighbors;
      neighbors.reserve(8);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {continue;}
          const float value = elevation[address(x + dx, y + dy)];
          if (value != kUnknown) {neighbors.push_back(value);}
        }
      }
      if (neighbors.size() >= 5) {
        const auto median = neighbors.begin() + neighbors.size() / 2;
        std::nth_element(neighbors.begin(), median, neighbors.end());
        filtered[index] = *median;
        variance[index] = 0.0F;
        roughness[index] = 0.0F;
      }
    }
  }
  elevation.swap(filtered);
}

void TerrainMapBuilder::computeTerrainFeatures(
  const std::vector<float> & elevation, const std::vector<float> & roughness,
  std::vector<float> & slope, std::vector<float> & traversability) const
{
  for (std::size_t y = 1; y + 1 < height_; ++y) {
    for (std::size_t x = 1; x + 1 < width_; ++x) {
      const std::size_t i = address(x, y);
      if (elevation[i] == kUnknown) {continue;}
      const float left = elevation[address(x - 1, y)];
      const float right = elevation[address(x + 1, y)];
      const float down = elevation[address(x, y - 1)];
      const float up = elevation[address(x, y + 1)];
      const bool has_left = left != kUnknown;
      const bool has_right = right != kUnknown;
      const bool has_down = down != kUnknown;
      const bool has_up = up != kUnknown;
      if ((!has_left && !has_right) || (!has_down && !has_up)) {continue;}

      // One-sided differences keep sparse LiDAR edges usable without inventing large surfaces.
      const double dzdx = ((has_right ? right : elevation[i]) -
        (has_left ? left : elevation[i])) /
        ((has_left && has_right) ? 2.0 * config_.resolution : config_.resolution);
      const double dzdy = ((has_up ? up : elevation[i]) -
        (has_down ? down : elevation[i])) /
        ((has_down && has_up) ? 2.0 * config_.resolution : config_.resolution);
      slope[i] = static_cast<float>(std::atan(std::hypot(dzdx, dzdy)));

      double max_step = 0.0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const float neighbor = elevation[address(x + dx, y + dy)];
          if (neighbor != kUnknown) {
            max_step = std::max(max_step, std::abs(static_cast<double>(neighbor - elevation[i])));
          }
        }
      }
      CellStatistics statistics;
      for (const auto & frame : cells_[i].frames) {
        statistics.min_z = std::min(statistics.min_z, frame.min_z);
        statistics.max_z = std::max(statistics.max_z, frame.max_z);
        statistics.count += frame.count;
      }
      const double vertical_span =
        statistics.count >= static_cast<std::uint32_t>(config_.min_points_per_cell) ?
        statistics.max_z - statistics.min_z : 0.0;
      if (vertical_span > config_.obstacle_height || max_step > config_.max_step_height) {
        traversability[i] = 0.0F;
        continue;
      }
      const double slope_score = std::clamp(1.0 - slope[i] / config_.max_slope, 0.0, 1.0);
      const double roughness_score = std::clamp(
        1.0 - roughness[i] / config_.max_roughness, 0.0, 1.0);
      traversability[i] = static_cast<float>(slope_score * roughness_score);
    }
  }
}

}  // namespace utree_dog_navigation
