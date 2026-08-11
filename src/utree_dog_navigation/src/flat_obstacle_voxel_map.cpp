#include "utree_dog_navigation/flat_obstacle_voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace utree_dog_navigation
{
namespace
{
constexpr std::size_t kNoVoxel = std::numeric_limits<std::size_t>::max();
constexpr double kRayEpsilon = 1.0e-10;

bool finitePoint(const TerrainPoint & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool clipAxis(
  double origin, double direction, double minimum, double maximum,
  double & enter, double & exit) noexcept
{
  if (std::abs(direction) <= kRayEpsilon) {
    return origin >= minimum && origin < maximum;
  }
  double first = (minimum - origin) / direction;
  double second = (maximum - origin) / direction;
  if (first > second) {
    std::swap(first, second);
  }
  enter = std::max(enter, first);
  exit = std::min(exit, second);
  return enter <= exit;
}

std::uint8_t incrementSaturated(std::uint8_t value) noexcept
{
  return value == std::numeric_limits<std::uint8_t>::max() ? value :
         static_cast<std::uint8_t>(value + 1U);
}
}  // namespace

FlatObstacleVoxelMap::FlatObstacleVoxelMap(
  TerrainMapConfig map_config, FlatObstacleVoxelConfig voxel_config)
: map_config_(std::move(map_config)), voxel_config_(std::move(voxel_config))
{
  if (!std::isfinite(map_config_.resolution) || map_config_.resolution <= 0.0 ||
    !std::isfinite(map_config_.size_x) || map_config_.size_x <= 0.0 ||
    !std::isfinite(map_config_.size_y) || map_config_.size_y <= 0.0 ||
    !std::isfinite(map_config_.origin_x) || !std::isfinite(map_config_.origin_y))
  {
    throw std::invalid_argument("flat obstacle map geometry is invalid");
  }
  if (!std::isfinite(voxel_config_.min_height) || voxel_config_.min_height < 0.0 ||
    !std::isfinite(voxel_config_.max_height) ||
    voxel_config_.max_height <= voxel_config_.min_height ||
    !std::isfinite(voxel_config_.voxel_height) || voxel_config_.voxel_height <= 0.0 ||
    !std::isfinite(voxel_config_.clearance) || voxel_config_.clearance < 0.0 ||
    voxel_config_.strong_hit_points == 0U ||
    voxel_config_.strong_hit_points > std::numeric_limits<std::uint16_t>::max() ||
    voxel_config_.hit_confirmation_frames == 0U ||
    voxel_config_.hit_confirmation_frames > std::numeric_limits<std::uint8_t>::max() ||
    !std::isfinite(voxel_config_.hit_confirmation_window) ||
    voxel_config_.hit_confirmation_window <= 0.0 ||
    voxel_config_.clear_confirmation_frames == 0U ||
    voxel_config_.clear_confirmation_frames > std::numeric_limits<std::uint8_t>::max() ||
    !std::isfinite(voxel_config_.clear_confirmation_window) ||
    voxel_config_.clear_confirmation_window <= 0.0)
  {
    throw std::invalid_argument("flat obstacle voxel configuration is invalid");
  }

  width_ = static_cast<std::size_t>(std::ceil(map_config_.size_x / map_config_.resolution));
  height_ = static_cast<std::size_t>(std::ceil(map_config_.size_y / map_config_.resolution));
  layers_ = static_cast<std::size_t>(std::ceil(
      (voxel_config_.max_height - voxel_config_.min_height) /
      voxel_config_.voxel_height));
  const std::size_t voxel_count = width_ * height_ * layers_;
  occupied_.assign(voxel_count, 0U);
  candidate_frames_.assign(voxel_count, 0U);
  clear_frames_.assign(voxel_count, 0U);
  frame_hits_.assign(voxel_count, 0U);
  frame_free_.assign(voxel_count, 0U);
  candidate_stamp_.assign(voxel_count, 0.0);
  clear_stamp_.assign(voxel_count, 0.0);
}

FlatObstacleUpdate FlatObstacleVoxelMap::update(
  const std::vector<TerrainPoint> & endpoints,
  const TerrainPoint & sensor_origin,
  double stamp_seconds, double ground_z, bool timing_continuous)
{
  FlatObstacleUpdate result;
  if (!finitePoint(sensor_origin) || !std::isfinite(stamp_seconds) ||
    !std::isfinite(ground_z))
  {
    result.confirmed_voxels = confirmed_voxel_count_;
    return result;
  }

  if (have_stamp_ && stamp_seconds == latest_stamp_seconds_) {
    result.confirmed_voxels = confirmed_voxel_count_;
    return result;
  }

  bool epoch_reset = false;
  if (have_stamp_ && stamp_seconds < latest_stamp_seconds_) {
    clearEpoch();
    epoch_reset = true;
  }

  const double frame_delta = have_stamp_ ? stamp_seconds - latest_stamp_seconds_ : 0.0;
  const bool continuous = have_stamp_ && timing_continuous &&
    frame_delta <= std::max(
    voxel_config_.hit_confirmation_window,
    voxel_config_.clear_confirmation_window);
  if (have_stamp_ && !continuous) {
    clearEpoch();
    epoch_reset = true;
  }

  std::fill(frame_hits_.begin(), frame_hits_.end(), 0U);
  std::fill(frame_free_.begin(), frame_free_.end(), 0U);
  for (const auto & endpoint : endpoints) {
    if (!finitePoint(endpoint)) {
      continue;
    }
    std::size_t x = 0U;
    std::size_t y = 0U;
    std::size_t z = 0U;
    std::size_t endpoint_index = kNoVoxel;
    if (toVoxel(endpoint, ground_z, x, y, z)) {
      endpoint_index = voxelAddress(x, y, z);
      if (frame_hits_[endpoint_index] == 0U) {
        ++result.endpoint_voxels;
      }
      if (frame_hits_[endpoint_index] < std::numeric_limits<std::uint16_t>::max()) {
        ++frame_hits_[endpoint_index];
      }
    }
    traceFreeVoxels(sensor_origin, endpoint, ground_z, endpoint_index);
  }

  for (std::size_t index = 0U; index < occupied_.size(); ++index) {
    if (frame_hits_[index] != 0U) {
      clear_frames_[index] = 0U;
      clear_stamp_[index] = 0.0;
      if (occupied_[index] != 0U) {
        continue;
      }
      if (frame_hits_[index] >= voxel_config_.strong_hit_points) {
        occupied_[index] = 1U;
        candidate_frames_[index] = 0U;
        candidate_stamp_[index] = 0.0;
        ++confirmed_voxel_count_;
        ++result.newly_confirmed_voxels;
        continue;
      }
      const bool extends_candidate = continuous && candidate_frames_[index] != 0U &&
        stamp_seconds - candidate_stamp_[index] <=
        voxel_config_.hit_confirmation_window;
      candidate_frames_[index] = extends_candidate ?
        incrementSaturated(candidate_frames_[index]) : 1U;
      candidate_stamp_[index] = stamp_seconds;
      if (candidate_frames_[index] >= voxel_config_.hit_confirmation_frames) {
        occupied_[index] = 1U;
        candidate_frames_[index] = 0U;
        candidate_stamp_[index] = 0.0;
        ++confirmed_voxel_count_;
        ++result.newly_confirmed_voxels;
      }
      continue;
    }

    if (candidate_frames_[index] != 0U &&
      (frame_free_[index] != 0U || !continuous ||
      stamp_seconds - candidate_stamp_[index] > voxel_config_.hit_confirmation_window))
    {
      candidate_frames_[index] = 0U;
      candidate_stamp_[index] = 0.0;
    }
    if (occupied_[index] == 0U) {
      continue;
    }
    if (continuous && frame_free_[index] != 0U) {
      const bool extends_clear = clear_frames_[index] != 0U &&
        stamp_seconds - clear_stamp_[index] <=
        voxel_config_.clear_confirmation_window;
      clear_frames_[index] = extends_clear ?
        incrementSaturated(clear_frames_[index]) : 1U;
      clear_stamp_[index] = stamp_seconds;
      if (clear_frames_[index] >= voxel_config_.clear_confirmation_frames) {
        occupied_[index] = 0U;
        clear_frames_[index] = 0U;
        clear_stamp_[index] = 0.0;
        --confirmed_voxel_count_;
        ++result.cleared_voxels;
      }
    } else {
      clear_frames_[index] = 0U;
      clear_stamp_[index] = 0.0;
    }
  }

  latest_stamp_seconds_ = stamp_seconds;
  have_stamp_ = true;
  result.accepted = true;
  result.epoch_reset = epoch_reset;
  result.confirmed_voxels = confirmed_voxel_count_;
  return result;
}

FlatObstacleSnapshot FlatObstacleVoxelMap::snapshot(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id, double ground_z,
  bool include_confirmed_voxel_centers) const
{
  FlatObstacleSnapshot result;
  result.raw_obstacles = projectRawObstacles();
  result.inflated_obstacles = inflate(result.raw_obstacles);

  auto & terrain = result.terrain;
  terrain.header.stamp = stamp;
  terrain.header.frame_id = frame_id;
  terrain.resolution = map_config_.resolution;
  terrain.width = static_cast<std::uint32_t>(width_);
  terrain.height = static_cast<std::uint32_t>(height_);
  terrain.origin_x = map_config_.origin_x;
  terrain.origin_y = map_config_.origin_y;
  terrain.unknown_value = TerrainMapBuilder::kUnknown;
  const std::size_t cell_count = width_ * height_;
  terrain.elevation.assign(cell_count, static_cast<float>(ground_z));
  terrain.variance.assign(cell_count, 0.0F);
  terrain.slope.assign(cell_count, 0.0F);
  terrain.roughness.assign(cell_count, 0.0F);
  terrain.traversability.assign(cell_count, 1.0F);
  terrain.observation_count.assign(cell_count, 1U);
  terrain.confidence.assign(cell_count, 1.0F);
  terrain.age.assign(cell_count, 0.0F);
  for (std::size_t index = 0U; index < cell_count; ++index) {
    if (result.raw_obstacles[index] != 0U) {
      terrain.traversability[index] = 0.0F;
    }
  }

  if (!include_confirmed_voxel_centers) {
    return result;
  }

  result.confirmed_voxel_centers.reserve(confirmed_voxel_count_);
  const std::size_t plane_size = width_ * height_;
  for (std::size_t index = 0U; index < occupied_.size(); ++index) {
    if (occupied_[index] == 0U) {
      continue;
    }
    const std::size_t z = index / plane_size;
    const std::size_t column = index % plane_size;
    const std::size_t y = column / width_;
    const std::size_t x = column % width_;
    const double slice_minimum = ground_z + voxel_config_.min_height +
      static_cast<double>(z) * voxel_config_.voxel_height;
    const double slice_maximum = std::min(
      ground_z + voxel_config_.max_height,
      slice_minimum + voxel_config_.voxel_height);
    result.confirmed_voxel_centers.push_back({
      map_config_.origin_x + (static_cast<double>(x) + 0.5) * map_config_.resolution,
      map_config_.origin_y + (static_cast<double>(y) + 0.5) * map_config_.resolution,
      0.5 * (slice_minimum + slice_maximum)});
  }
  return result;
}

void FlatObstacleVoxelMap::resetEpoch()
{
  clearEpoch();
}

std::size_t FlatObstacleVoxelMap::width() const noexcept {return width_;}
std::size_t FlatObstacleVoxelMap::height() const noexcept {return height_;}
std::size_t FlatObstacleVoxelMap::layers() const noexcept {return layers_;}
const TerrainMapConfig & FlatObstacleVoxelMap::mapConfig() const noexcept
{
  return map_config_;
}
const FlatObstacleVoxelConfig & FlatObstacleVoxelMap::voxelConfig() const noexcept
{
  return voxel_config_;
}

bool FlatObstacleVoxelMap::toVoxel(
  const TerrainPoint & point, double ground_z,
  std::size_t & x, std::size_t & y, std::size_t & z) const
{
  const double relative_z = point.z - ground_z;
  if (!finitePoint(point) || !std::isfinite(relative_z) ||
    relative_z < voxel_config_.min_height || relative_z > voxel_config_.max_height)
  {
    return false;
  }
  const int grid_x = static_cast<int>(std::floor(
      (point.x - map_config_.origin_x) / map_config_.resolution));
  const int grid_y = static_cast<int>(std::floor(
      (point.y - map_config_.origin_y) / map_config_.resolution));
  const int grid_z = std::min(
    static_cast<int>(layers_) - 1,
    static_cast<int>(std::floor(
      (relative_z - voxel_config_.min_height) / voxel_config_.voxel_height)));
  if (grid_x < 0 || grid_y < 0 || grid_z < 0 ||
    grid_x >= static_cast<int>(width_) || grid_y >= static_cast<int>(height_) ||
    grid_z >= static_cast<int>(layers_))
  {
    return false;
  }
  x = static_cast<std::size_t>(grid_x);
  y = static_cast<std::size_t>(grid_y);
  z = static_cast<std::size_t>(grid_z);
  return true;
}

void FlatObstacleVoxelMap::traceFreeVoxels(
  const TerrainPoint & sensor_origin, const TerrainPoint & endpoint,
  double ground_z, std::size_t endpoint_index)
{
  const TerrainPoint direction{
    endpoint.x - sensor_origin.x,
    endpoint.y - sensor_origin.y,
    endpoint.z - sensor_origin.z};
  if (std::abs(direction.x) <= kRayEpsilon &&
    std::abs(direction.y) <= kRayEpsilon &&
    std::abs(direction.z) <= kRayEpsilon)
  {
    return;
  }

  double enter = 0.0;
  double exit = 1.0;
  const double maximum_x = map_config_.origin_x +
    static_cast<double>(width_) * map_config_.resolution;
  const double maximum_y = map_config_.origin_y +
    static_cast<double>(height_) * map_config_.resolution;
  const double minimum_z = ground_z + voxel_config_.min_height;
  const double maximum_z = ground_z + voxel_config_.max_height;
  if (!clipAxis(
      sensor_origin.x, direction.x, map_config_.origin_x, maximum_x, enter, exit) ||
    !clipAxis(
      sensor_origin.y, direction.y, map_config_.origin_y, maximum_y, enter, exit) ||
    !clipAxis(sensor_origin.z, direction.z, minimum_z, maximum_z, enter, exit) ||
    enter > exit || exit < 0.0 || enter > 1.0)
  {
    return;
  }
  enter = std::max(0.0, enter);
  exit = std::min(1.0, exit);
  if (enter > exit) {
    return;
  }

  const double start_t = std::nextafter(enter, exit);
  const double finish_t = std::nextafter(exit, enter);
  const TerrainPoint start{
    sensor_origin.x + direction.x * start_t,
    sensor_origin.y + direction.y * start_t,
    sensor_origin.z + direction.z * start_t};
  const TerrainPoint finish{
    sensor_origin.x + direction.x * finish_t,
    sensor_origin.y + direction.y * finish_t,
    sensor_origin.z + direction.z * finish_t};
  std::size_t start_x = 0U;
  std::size_t start_y = 0U;
  std::size_t start_z = 0U;
  std::size_t finish_x = 0U;
  std::size_t finish_y = 0U;
  std::size_t finish_z = 0U;
  if (!toVoxel(start, ground_z, start_x, start_y, start_z) ||
    !toVoxel(finish, ground_z, finish_x, finish_y, finish_z))
  {
    return;
  }

  int x = static_cast<int>(start_x);
  int y = static_cast<int>(start_y);
  int z = static_cast<int>(start_z);
  const int target_x = static_cast<int>(finish_x);
  const int target_y = static_cast<int>(finish_y);
  const int target_z = static_cast<int>(finish_z);
  const int step_x = direction.x > kRayEpsilon ? 1 :
    (direction.x < -kRayEpsilon ? -1 : 0);
  const int step_y = direction.y > kRayEpsilon ? 1 :
    (direction.y < -kRayEpsilon ? -1 : 0);
  const int step_z = direction.z > kRayEpsilon ? 1 :
    (direction.z < -kRayEpsilon ? -1 : 0);
  const double infinity = std::numeric_limits<double>::infinity();
  const auto nextCrossing = [](
      double origin, double direction_value, double grid_origin,
      double cell_size, int coordinate, int step) {
      if (step == 0) {
        return std::numeric_limits<double>::infinity();
      }
      const double boundary = grid_origin +
        static_cast<double>(coordinate + (step > 0 ? 1 : 0)) * cell_size;
      return (boundary - origin) / direction_value;
    };
  double next_x = nextCrossing(
    sensor_origin.x, direction.x, map_config_.origin_x,
    map_config_.resolution, x, step_x);
  double next_y = nextCrossing(
    sensor_origin.y, direction.y, map_config_.origin_y,
    map_config_.resolution, y, step_y);
  double next_z = nextCrossing(
    sensor_origin.z, direction.z, minimum_z,
    voxel_config_.voxel_height, z, step_z);
  const double delta_x = step_x == 0 ? infinity :
    map_config_.resolution / std::abs(direction.x);
  const double delta_y = step_y == 0 ? infinity :
    map_config_.resolution / std::abs(direction.y);
  const double delta_z = step_z == 0 ? infinity :
    voxel_config_.voxel_height / std::abs(direction.z);

  const std::size_t maximum_steps = width_ + height_ + layers_ + 3U;
  for (std::size_t step = 0U; step < maximum_steps; ++step) {
    if (x < 0 || y < 0 || z < 0 || x >= static_cast<int>(width_) ||
      y >= static_cast<int>(height_) || z >= static_cast<int>(layers_))
    {
      break;
    }
    const std::size_t index = voxelAddress(
      static_cast<std::size_t>(x), static_cast<std::size_t>(y),
      static_cast<std::size_t>(z));
    if (index != endpoint_index) {
      frame_free_[index] = 1U;
    }
    if (x == target_x && y == target_y && z == target_z) {
      break;
    }
    const double next = std::min({next_x, next_y, next_z});
    if (!std::isfinite(next) || next > exit + kRayEpsilon) {
      break;
    }
    if (next_x <= next + kRayEpsilon) {
      x += step_x;
      next_x += delta_x;
    }
    if (next_y <= next + kRayEpsilon) {
      y += step_y;
      next_y += delta_y;
    }
    if (next_z <= next + kRayEpsilon) {
      z += step_z;
      next_z += delta_z;
    }
  }
}

std::size_t FlatObstacleVoxelMap::voxelAddress(
  std::size_t x, std::size_t y, std::size_t z) const noexcept
{
  return z * width_ * height_ + columnAddress(x, y);
}

std::size_t FlatObstacleVoxelMap::columnAddress(
  std::size_t x, std::size_t y) const noexcept
{
  return y * width_ + x;
}

std::vector<std::uint8_t> FlatObstacleVoxelMap::projectRawObstacles() const
{
  std::vector<std::uint8_t> raw(width_ * height_, 0U);
  const std::size_t plane_size = width_ * height_;
  for (std::size_t index = 0U; index < occupied_.size(); ++index) {
    if (occupied_[index] != 0U) {
      raw[index % plane_size] = 1U;
    }
  }
  return raw;
}

std::vector<std::uint8_t> FlatObstacleVoxelMap::inflate(
  const std::vector<std::uint8_t> & raw) const
{
  std::vector<std::uint8_t> inflated = raw;
  if (voxel_config_.clearance <= 0.0) {
    return inflated;
  }
  const int maximum_offset =
    static_cast<int>(std::ceil(voxel_config_.clearance / map_config_.resolution)) + 1;
  constexpr double kDistanceEpsilon = 1.0e-12;
  for (std::size_t y = 0U; y < height_; ++y) {
    for (std::size_t x = 0U; x < width_; ++x) {
      if (raw[columnAddress(x, y)] == 0U) {
        continue;
      }
      for (int dy = -maximum_offset; dy <= maximum_offset; ++dy) {
        for (int dx = -maximum_offset; dx <= maximum_offset; ++dx) {
          const int candidate_x = static_cast<int>(x) + dx;
          const int candidate_y = static_cast<int>(y) + dy;
          if (candidate_x < 0 || candidate_y < 0 ||
            candidate_x >= static_cast<int>(width_) ||
            candidate_y >= static_cast<int>(height_))
          {
            continue;
          }
          const double gap_x =
            std::max(0, std::abs(dx) - 1) * map_config_.resolution;
          const double gap_y =
            std::max(0, std::abs(dy) - 1) * map_config_.resolution;
          if (std::hypot(gap_x, gap_y) <= voxel_config_.clearance + kDistanceEpsilon) {
            inflated[columnAddress(
                static_cast<std::size_t>(candidate_x),
                static_cast<std::size_t>(candidate_y))] = 1U;
          }
        }
      }
    }
  }
  return inflated;
}

void FlatObstacleVoxelMap::clearTransientEvidence()
{
  std::fill(candidate_frames_.begin(), candidate_frames_.end(), 0U);
  std::fill(clear_frames_.begin(), clear_frames_.end(), 0U);
  std::fill(candidate_stamp_.begin(), candidate_stamp_.end(), 0.0);
  std::fill(clear_stamp_.begin(), clear_stamp_.end(), 0.0);
}

void FlatObstacleVoxelMap::clearEpoch()
{
  std::fill(occupied_.begin(), occupied_.end(), 0U);
  std::fill(frame_hits_.begin(), frame_hits_.end(), 0U);
  std::fill(frame_free_.begin(), frame_free_.end(), 0U);
  clearTransientEvidence();
  confirmed_voxel_count_ = 0U;
  have_stamp_ = false;
}

}  // namespace utree_dog_navigation
