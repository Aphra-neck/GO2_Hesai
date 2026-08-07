#include "utree_dog_navigation/lattice_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace utree_dog_navigation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDistanceTolerance = 1.0e-6;
constexpr double kMaximumIntegerSafeDistance =
  static_cast<double>(std::numeric_limits<int>::max());

double normalizeAngle(double angle)
{
  double normalized = std::remainder(angle, 2.0 * kPi);
  if (normalized <= -kPi) {normalized += 2.0 * kPi;}
  return normalized;
}

struct SearchRecord
{
  double g{std::numeric_limits<double>::infinity()};
  std::uint64_t parent{0};
  bool has_parent{false};
  bool closed{false};
};

struct QueueItem
{
  double f{0.0};
  double g{0.0};
  std::uint64_t key{0};
  bool operator<(const QueueItem & other) const {return f > other.f;}
};

bool finiteWorldState(const WorldState & state)
{
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.yaw);
}

bool solveThreeByThree(
  std::array<std::array<double, 4>, 3> matrix, std::array<double, 3> & solution)
{
  for (std::size_t column = 0; column < 3; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1; row < 3; ++row) {
      if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][column]) < 1.0e-10) {return false;}
    if (pivot != column) {std::swap(matrix[pivot], matrix[column]);}

    const double divisor = matrix[column][column];
    for (std::size_t item = column; item < 4; ++item) {
      matrix[column][item] /= divisor;
    }
    for (std::size_t row = 0; row < 3; ++row) {
      if (row == column) {continue;}
      const double factor = matrix[row][column];
      for (std::size_t item = column; item < 4; ++item) {
        matrix[row][item] -= factor * matrix[column][item];
      }
    }
  }
  for (std::size_t row = 0; row < 3; ++row) {
    solution[row] = matrix[row][3];
    if (!std::isfinite(solution[row])) {return false;}
  }
  return true;
}
}  // namespace

std::string_view verifiedFlatStartStatusName(VerifiedFlatStartStatus status) noexcept
{
  switch (status) {
    case VerifiedFlatStartStatus::kNotNeeded: return "not_needed";
    case VerifiedFlatStartStatus::kDisabled: return "disabled";
    case VerifiedFlatStartStatus::kApplied: return "applied";
    case VerifiedFlatStartStatus::kInvalidConfiguration: return "invalid_configuration";
    case VerifiedFlatStartStatus::kMissingObservationLayer: return "missing_observation_layer";
    case VerifiedFlatStartStatus::kInsufficientSupport: return "insufficient_support";
    case VerifiedFlatStartStatus::kInsufficientSectors: return "insufficient_sectors";
    case VerifiedFlatStartStatus::kPlaneFitFailed: return "plane_fit_failed";
    case VerifiedFlatStartStatus::kSupportNotFlat: return "support_not_flat";
    case VerifiedFlatStartStatus::kNoInferredStartCell: return "no_inferred_start_cell";
    case VerifiedFlatStartStatus::kNoObservedConnection: return "no_observed_connection";
  }
  return "unknown";
}

LatticePlanner::LatticePlanner(LatticePlannerConfig config) : config_(std::move(config))
{
  config_.yaw_bins = std::max(8, config_.yaw_bins);
  if (config_.start_snap_radius < 0.0) {
    config_.start_snap_radius = config_.snap_radius;
  }
  const bool valid_base_config = config_.yaw_bins <= 360 &&
    std::isfinite(config_.motion_step) && config_.motion_step > 0.0 &&
    config_.motion_step <= kMaximumIntegerSafeDistance &&
    std::isfinite(config_.min_traversability) && config_.min_traversability >= 0.0 &&
    config_.min_traversability <= 1.0 &&
    std::isfinite(config_.max_step_height) && config_.max_step_height >= 0.0 &&
    std::isfinite(config_.max_slope) && config_.max_slope >= 0.0 && config_.max_slope <= kPi / 2.0 &&
    std::isfinite(config_.stair_height_threshold) && config_.stair_height_threshold >= 0.0 &&
    config_.stair_height_threshold <= config_.max_step_height &&
    std::isfinite(config_.terrain_cost_weight) && config_.terrain_cost_weight >= 0.0 &&
    std::isfinite(config_.slope_cost_weight) && config_.slope_cost_weight >= 0.0 &&
    std::isfinite(config_.height_cost_weight) && config_.height_cost_weight >= 0.0 &&
    std::isfinite(config_.yaw_change_cost) && config_.yaw_change_cost >= 0.0 &&
    std::isfinite(config_.reverse_cost_factor) && config_.reverse_cost_factor > 0.0 &&
    std::isfinite(config_.lateral_cost_factor) && config_.lateral_cost_factor > 0.0 &&
    config_.max_expansions > 0 && std::isfinite(config_.snap_radius) &&
    config_.snap_radius >= 0.0 && config_.snap_radius <= kMaximumIntegerSafeDistance &&
    std::isfinite(config_.start_snap_radius) && config_.start_snap_radius >= 0.0 &&
    config_.start_snap_radius <= kMaximumIntegerSafeDistance;
  if (!valid_base_config) {throw std::invalid_argument("lattice planner configuration is invalid");}
  if (config_.verified_flat_start.enabled && !verifiedFlatConfigurationValid()) {
    throw std::invalid_argument("verified flat start configuration is invalid");
  }
}

void LatticePlanner::setMap(utree_dog_msgs::msg::TerrainGrid::SharedPtr map)
{
  map_ = std::move(map);
}

bool LatticePlanner::hasMap() const noexcept {return static_cast<bool>(map_);}

bool LatticePlanner::mapValid() const
{
  if (!map_ || !std::isfinite(map_->resolution) || map_->resolution <= 0.0F ||
    !std::isfinite(map_->origin_x) || !std::isfinite(map_->origin_y) ||
    !std::isfinite(map_->unknown_value) || map_->width == 0 || map_->height == 0 ||
    map_->width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    map_->height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
  {
    return false;
  }
  const std::size_t expected = static_cast<std::size_t>(map_->width) * map_->height;
  if (map_->elevation.size() != expected || map_->slope.size() != expected ||
    map_->traversability.size() != expected)
  {
    return false;
  }
  for (std::size_t index = 0; index < expected; ++index) {
    const float elevation = map_->elevation[index];
    const float slope = map_->slope[index];
    const float traversability = map_->traversability[index];
    if (!std::isfinite(elevation) || !std::isfinite(slope) ||
      !std::isfinite(traversability))
    {
      return false;
    }
    if (slope != map_->unknown_value && slope < 0.0F) {return false;}
    if (traversability != map_->unknown_value &&
      (traversability < 0.0F || traversability > 1.0F))
    {
      return false;
    }
  }
  return true;
}

PlanningResult LatticePlanner::plan(
  const WorldState & start_world, const WorldState & goal_world,
  const std::function<bool()> & cancellation_requested) const
{
  PlanningResult result;
  if (!mapValid() || !finiteWorldState(start_world) || !finiteWorldState(goal_world)) {
    return result;
  }

  SearchState start;
  GridState goal;
  if (!toGrid(start_world.x, start_world.y, start.grid.x, start.grid.y) ||
    !toGrid(goal_world.x, goal_world.y, goal.x, goal.y)) {return result;}
  start.grid.yaw = yawBin(start_world.yaw);
  goal.yaw = yawBin(goal_world.yaw);

  PlanningOverlay overlay;
  if (!nearestObservedValid(
      start_world.x, start_world.y, config_.start_snap_radius,
      start.grid.x, start.grid.y))
  {
    if (!config_.verified_flat_start.enabled) {
      result.start_status = VerifiedFlatStartStatus::kDisabled;
      return result;
    }
    result.start_status = buildVerifiedFlatOverlay(start_world.x, start_world.y, overlay);
    if (result.start_status != VerifiedFlatStartStatus::kApplied) {return result;}
    if (!overlayCell(start.grid.x, start.grid.y, overlay)) {
      result.start_status = VerifiedFlatStartStatus::kNoInferredStartCell;
      return result;
    }
    start.inferred_prefix = true;
    if (!inferredStartConnectsToObserved(start, overlay)) {
      result.start_status = VerifiedFlatStartStatus::kNoObservedConnection;
      return result;
    }
    result.exact_start_inferred = true;
    result.exact_start_elevation = overlay.plane_z;
    result.exact_start_dzdx = overlay.plane_x;
    result.exact_start_dzdy = overlay.plane_y;
  }
  if (!nearestObservedValid(
      goal_world.x, goal_world.y, config_.snap_radius, goal.x, goal.y)) {return result;}

  const auto planner_motions = motions();

  // Sparse records keep memory proportional to explored states rather than map size * yaw bins.
  std::priority_queue<QueueItem> open;
  std::unordered_map<std::uint64_t, SearchRecord> records;
  const std::uint64_t start_key = key(start);
  records[start_key].g = 0.0;
  open.push({heuristic(start.grid, goal), 0.0, start_key});
  std::uint64_t reached_key = 0;

  while (!open.empty() && result.expansions < config_.max_expansions) {
    if (cancellation_requested && cancellation_requested()) {return result;}
    const QueueItem item = open.top();
    open.pop();
    auto current_record = records.find(item.key);
    if (current_record == records.end() || current_record->second.closed ||
      item.g > current_record->second.g) {continue;}
    current_record->second.closed = true;
    const double current_g = current_record->second.g;
    const SearchState current = decode(item.key);
    ++result.expansions;
    if (!current.inferred_prefix && current.grid.x == goal.x && current.grid.y == goal.y &&
      current.grid.yaw == goal.yaw)
    {
      reached_key = item.key;
      result.success = true;
      break;
    }

    for (const auto & motion : planner_motions) {
      SearchState next;
      double edge_cost = 0.0;
      if (!transition(current, motion, overlay, next, edge_cost)) {continue;}
      const std::uint64_t next_key = key(next);
      const double next_g = current_g + edge_cost;
      auto & next_record = records[next_key];
      if (next_record.closed || next_g >= next_record.g) {continue;}
      next_record.g = next_g;
      next_record.parent = item.key;
      next_record.has_parent = true;
      open.push({next_g + heuristic(next.grid, goal), next_g, next_key});
    }
  }

  if (!result.success) {return result;}
  for (std::uint64_t cursor = reached_key;;) {
    result.states.push_back(plannedState(decode(cursor), overlay));
    const auto & record = records.at(cursor);
    if (!record.has_parent) {break;}
    cursor = record.parent;
  }
  std::reverse(result.states.begin(), result.states.end());
  return result;
}

double LatticePlanner::yawAngle(int bin) const
{
  return normalizeAngle(2.0 * kPi * static_cast<double>(bin) / config_.yaw_bins);
}

double LatticePlanner::elevationAt(int x, int y, double fallback) const
{
  if (!inside(x, y)) {return fallback;}
  const float value = map_->elevation[cellAddress(x, y)];
  return value == map_->unknown_value ? fallback : value;
}

const utree_dog_msgs::msg::TerrainGrid & LatticePlanner::map() const
{
  if (!map_) {throw std::logic_error("terrain map is not set");}
  return *map_;
}

std::array<LatticePlanner::Motion, 10> LatticePlanner::motions() const
{
  return {{
    {1.0, 0.0, 0, 1.0}, {-1.0, 0.0, 0, config_.reverse_cost_factor},
    {0.0, 1.0, 0, config_.lateral_cost_factor},
    {0.0, -1.0, 0, config_.lateral_cost_factor},
    {0.7071, 0.7071, 0, 1.1}, {0.7071, -0.7071, 0, 1.1},
    {-0.7071, 0.7071, 0, 1.3}, {-0.7071, -0.7071, 0, 1.3},
    {0.0, 0.0, 1, 1.0}, {0.0, 0.0, -1, 1.0}}};
}

bool LatticePlanner::toGrid(double x, double y, int & gx, int & gy) const
{
  const double grid_x = (x - map_->origin_x) / map_->resolution;
  const double grid_y = (y - map_->origin_y) / map_->resolution;
  if (!std::isfinite(grid_x) || !std::isfinite(grid_y) || grid_x < 0.0 || grid_y < 0.0 ||
    grid_x >= static_cast<double>(map_->width) || grid_y >= static_cast<double>(map_->height))
  {
    return false;
  }
  gx = static_cast<int>(std::floor(grid_x));
  gy = static_cast<int>(std::floor(grid_y));
  return true;
}

bool LatticePlanner::inside(int x, int y) const
{
  return x >= 0 && y >= 0 && x < static_cast<int>(map_->width) &&
         y < static_cast<int>(map_->height);
}

std::size_t LatticePlanner::cellAddress(int x, int y) const
{
  return static_cast<std::size_t>(y) * map_->width + static_cast<std::size_t>(x);
}

std::uint64_t LatticePlanner::key(const SearchState & state) const
{
  const std::uint64_t grid_key =
    static_cast<std::uint64_t>(cellAddress(state.grid.x, state.grid.y)) * config_.yaw_bins +
    state.grid.yaw;
  return grid_key * 2U + static_cast<std::uint64_t>(state.inferred_prefix);
}

LatticePlanner::SearchState LatticePlanner::decode(std::uint64_t value) const
{
  SearchState state;
  state.inferred_prefix = value % 2U != 0U;
  value /= 2U;
  state.grid.yaw = static_cast<int>(value % config_.yaw_bins);
  const std::uint64_t cell = value / config_.yaw_bins;
  state.grid.x = static_cast<int>(cell % map_->width);
  state.grid.y = static_cast<int>(cell / map_->width);
  return state;
}

int LatticePlanner::yawBin(double yaw) const
{
  const double positive = normalizeAngle(yaw) + kPi;
  int bin = static_cast<int>(std::lround(positive * config_.yaw_bins / (2.0 * kPi)));
  return (bin + config_.yaw_bins / 2) % config_.yaw_bins;
}

bool LatticePlanner::observedValidCell(int x, int y) const
{
  if (!inside(x, y)) {return false;}
  const std::size_t i = cellAddress(x, y);
  return map_->elevation[i] != map_->unknown_value &&
         map_->traversability[i] != map_->unknown_value &&
         map_->traversability[i] >= config_.min_traversability &&
         map_->slope[i] != map_->unknown_value && map_->slope[i] <= config_.max_slope;
}

bool LatticePlanner::nearestObservedValid(
  double world_x, double world_y, double snap_radius, int & x, int & y) const
{
  if (observedValidCell(x, y)) {return true;}
  const double radius_cells = std::ceil(snap_radius / map_->resolution);
  const int map_max_x = static_cast<int>(map_->width) - 1;
  const int map_max_y = static_cast<int>(map_->height) - 1;
  int minimum_x = 0;
  int maximum_x = map_max_x;
  int minimum_y = 0;
  int maximum_y = map_max_y;
  const int largest_dimension = std::max(map_max_x, map_max_y) + 1;
  if (std::isfinite(radius_cells) && radius_cells < largest_dimension) {
    const int radius = std::max(1, static_cast<int>(radius_cells));
    minimum_x = std::max(0, x - radius);
    maximum_x = static_cast<int>(std::min<std::int64_t>(
        map_max_x, static_cast<std::int64_t>(x) + radius));
    minimum_y = std::max(0, y - radius);
    maximum_y = static_cast<int>(std::min<std::int64_t>(
        map_max_y, static_cast<std::int64_t>(y) + radius));
  }
  int best_x = x;
  int best_y = y;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::int64_t candidate_y_64 = minimum_y; candidate_y_64 <= maximum_y;
    ++candidate_y_64)
  {
    const int candidate_y = static_cast<int>(candidate_y_64);
    for (std::int64_t candidate_x_64 = minimum_x; candidate_x_64 <= maximum_x;
      ++candidate_x_64)
    {
      const int candidate_x = static_cast<int>(candidate_x_64);
      if (!observedValidCell(candidate_x, candidate_y)) {continue;}
      const double candidate_world_x =
        map_->origin_x + (static_cast<double>(candidate_x) + 0.5) * map_->resolution;
      const double candidate_world_y =
        map_->origin_y + (static_cast<double>(candidate_y) + 0.5) * map_->resolution;
      const double distance_m = std::hypot(
        candidate_world_x - world_x, candidate_world_y - world_y);
      if (distance_m > snap_radius + kDistanceTolerance) {continue;}
      if (distance_m < best_distance) {
        best_x = candidate_x;
        best_y = candidate_y;
        best_distance = distance_m;
      }
    }
  }
  if (!std::isfinite(best_distance)) {return false;}
  x = best_x;
  y = best_y;
  return true;
}

bool LatticePlanner::verifiedFlatConfigurationValid() const
{
  const auto & config = config_.verified_flat_start;
  return
    std::isfinite(config.support_inner_radius) && config.support_inner_radius >= 0.0 &&
    std::isfinite(config.support_outer_radius) &&
    config.support_outer_radius > config.support_inner_radius &&
    std::isfinite(config.fill_radius) && config.fill_radius > 0.0 &&
    config.fill_radius <= config.support_outer_radius &&
    config.fill_radius + config_.motion_step + kDistanceTolerance >=
    config.support_inner_radius &&
    config.sector_count >= 4 && config.sector_count <= 64 &&
    config.min_supported_sectors > 0 &&
    config.min_supported_sectors <= config.sector_count &&
    config.min_cells_per_sector > 0 && config.min_support_cells > 0 &&
    config.min_observation_count > 0 && config.min_observation_count <= 65535 &&
    std::isfinite(config.max_plane_slope) && config.max_plane_slope >= 0.0 &&
    config.max_plane_slope <= config_.max_slope &&
    std::isfinite(config.max_plane_rmse) && config.max_plane_rmse >= 0.0 &&
    std::isfinite(config.max_plane_residual) && config.max_plane_residual >= 0.0 &&
    std::isfinite(config.max_elevation_range) && config.max_elevation_range >= 0.0 &&
    std::isfinite(config.inferred_traversability) &&
    config.inferred_traversability >= config_.min_traversability &&
    config.inferred_traversability <= 1.0;
}

VerifiedFlatStartStatus LatticePlanner::buildVerifiedFlatOverlay(
  double start_x, double start_y, PlanningOverlay & overlay) const
{
  const auto & config = config_.verified_flat_start;
  if (!verifiedFlatConfigurationValid()) {
    return VerifiedFlatStartStatus::kInvalidConfiguration;
  }

  const std::size_t cell_count = static_cast<std::size_t>(map_->width) * map_->height;
  if (map_->observation_count.size() != cell_count) {
    return VerifiedFlatStartStatus::kMissingObservationLayer;
  }

  struct SupportSample
  {
    double x;
    double y;
    double z;
  };
  std::vector<SupportSample> samples;
  std::vector<int> sector_cells(static_cast<std::size_t>(config.sector_count), 0);
  double minimum_elevation = std::numeric_limits<double>::infinity();
  double maximum_elevation = -std::numeric_limits<double>::infinity();

  for (int y = 0; y < static_cast<int>(map_->height); ++y) {
    for (int x = 0; x < static_cast<int>(map_->width); ++x) {
      if (!observedValidCell(x, y)) {continue;}
      const std::size_t index = cellAddress(x, y);
      if (map_->observation_count[index] < config.min_observation_count) {continue;}
      const double world_x = map_->origin_x + (static_cast<double>(x) + 0.5) * map_->resolution;
      const double world_y = map_->origin_y + (static_cast<double>(y) + 0.5) * map_->resolution;
      const double dx = world_x - start_x;
      const double dy = world_y - start_y;
      const double radius = std::hypot(dx, dy);
      if (radius + kDistanceTolerance < config.support_inner_radius ||
        radius > config.support_outer_radius + kDistanceTolerance)
      {
        continue;
      }

      double angle = std::atan2(dy, dx);
      if (angle < 0.0) {angle += 2.0 * kPi;}
      const int sector = std::min(
        config.sector_count - 1,
        static_cast<int>(angle * config.sector_count / (2.0 * kPi)));
      ++sector_cells[static_cast<std::size_t>(sector)];
      const double elevation = map_->elevation[index];
      samples.push_back({dx, dy, elevation});
      minimum_elevation = std::min(minimum_elevation, elevation);
      maximum_elevation = std::max(maximum_elevation, elevation);
    }
  }

  if (samples.size() < static_cast<std::size_t>(config.min_support_cells)) {
    return VerifiedFlatStartStatus::kInsufficientSupport;
  }
  const int supported_sectors = static_cast<int>(std::count_if(
      sector_cells.begin(), sector_cells.end(),
      [&config](int count) {return count >= config.min_cells_per_sector;}));
  if (supported_sectors < config.min_supported_sectors) {
    return VerifiedFlatStartStatus::kInsufficientSectors;
  }
  if (maximum_elevation - minimum_elevation > config.max_elevation_range) {
    return VerifiedFlatStartStatus::kSupportNotFlat;
  }

  std::array<std::array<double, 4>, 3> normal{};
  for (const auto & sample : samples) {
    normal[0][0] += sample.x * sample.x;
    normal[0][1] += sample.x * sample.y;
    normal[0][2] += sample.x;
    normal[0][3] += sample.x * sample.z;
    normal[1][0] += sample.x * sample.y;
    normal[1][1] += sample.y * sample.y;
    normal[1][2] += sample.y;
    normal[1][3] += sample.y * sample.z;
    normal[2][0] += sample.x;
    normal[2][1] += sample.y;
    normal[2][2] += 1.0;
    normal[2][3] += sample.z;
  }
  std::array<double, 3> plane{};
  if (!solveThreeByThree(normal, plane)) {
    return VerifiedFlatStartStatus::kPlaneFitFailed;
  }
  const double plane_slope = std::atan(std::hypot(plane[0], plane[1]));
  if (!std::isfinite(plane_slope) || plane_slope > config.max_plane_slope) {
    return VerifiedFlatStartStatus::kSupportNotFlat;
  }

  double squared_residual_sum = 0.0;
  double maximum_residual = 0.0;
  for (const auto & sample : samples) {
    const double predicted = plane[0] * sample.x + plane[1] * sample.y + plane[2];
    const double residual = std::abs(sample.z - predicted);
    squared_residual_sum += residual * residual;
    maximum_residual = std::max(maximum_residual, residual);
  }
  const double rmse = std::sqrt(squared_residual_sum / static_cast<double>(samples.size()));
  if (!std::isfinite(rmse) || rmse > config.max_plane_rmse ||
    maximum_residual > config.max_plane_residual)
  {
    return VerifiedFlatStartStatus::kSupportNotFlat;
  }

  overlay.active = true;
  overlay.center_x = start_x;
  overlay.center_y = start_y;
  overlay.plane_x = plane[0];
  overlay.plane_y = plane[1];
  overlay.plane_z = plane[2];
  overlay.slope = plane_slope;
  overlay.traversability = config.inferred_traversability;
  overlay.inferred_cells.assign(cell_count, 0U);
  for (int y = 0; y < static_cast<int>(map_->height); ++y) {
    for (int x = 0; x < static_cast<int>(map_->width); ++x) {
      const std::size_t index = cellAddress(x, y);
      const double world_x = map_->origin_x + (static_cast<double>(x) + 0.5) * map_->resolution;
      const double world_y = map_->origin_y + (static_cast<double>(y) + 0.5) * map_->resolution;
      if (std::hypot(world_x - start_x, world_y - start_y) >
        config.fill_radius + kDistanceTolerance)
      {
        continue;
      }
      const bool completely_unobserved = map_->observation_count[index] == 0U &&
        map_->elevation[index] == map_->unknown_value &&
        map_->slope[index] == map_->unknown_value &&
        map_->traversability[index] == map_->unknown_value;
      if (completely_unobserved) {overlay.inferred_cells[index] = 1U;}
    }
  }
  return VerifiedFlatStartStatus::kApplied;
}

bool LatticePlanner::overlayCell(int x, int y, const PlanningOverlay & overlay) const
{
  return overlay.active && inside(x, y) &&
         overlay.inferred_cells[cellAddress(x, y)] != 0U;
}

bool LatticePlanner::inferredStartConnectsToObserved(
  const SearchState & start, const PlanningOverlay & overlay) const
{
  const auto planner_motions = motions();
  std::queue<SearchState> pending;
  std::unordered_set<std::uint64_t> visited;
  pending.push(start);
  visited.insert(key(start));
  while (!pending.empty()) {
    const SearchState current = pending.front();
    pending.pop();
    for (const auto & motion : planner_motions) {
      SearchState next;
      double transition_cost = 0.0;
      if (!transition(current, motion, overlay, next, transition_cost)) {continue;}
      if (!next.inferred_prefix) {return true;}
      if (!visited.insert(key(next)).second) {continue;}
      pending.push(next);
    }
  }
  return false;
}

bool LatticePlanner::cellProperties(
  int x, int y, const PlanningOverlay & overlay, bool allow_inferred,
  CellProperties & properties) const
{
  if (observedValidCell(x, y)) {
    const std::size_t index = cellAddress(x, y);
    properties.elevation = map_->elevation[index];
    properties.slope = map_->slope[index];
    properties.traversability = map_->traversability[index];
    properties.inferred = false;
    return true;
  }
  if (!allow_inferred || !overlayCell(x, y, overlay)) {return false;}
  const double world_x = map_->origin_x + (static_cast<double>(x) + 0.5) * map_->resolution;
  const double world_y = map_->origin_y + (static_cast<double>(y) + 0.5) * map_->resolution;
  properties.elevation = overlay.plane_x * (world_x - overlay.center_x) +
    overlay.plane_y * (world_y - overlay.center_y) + overlay.plane_z;
  properties.slope = overlay.slope;
  properties.traversability = overlay.traversability;
  properties.inferred = true;
  return std::isfinite(properties.elevation);
}

double LatticePlanner::surfaceElevation(
  int x, int y, const PlanningOverlay & overlay, double fallback) const
{
  if (!inside(x, y)) {return fallback;}
  const std::size_t index = cellAddress(x, y);
  if (map_->elevation[index] != map_->unknown_value) {return map_->elevation[index];}
  CellProperties properties;
  return cellProperties(x, y, overlay, true, properties) ? properties.elevation : fallback;
}

double LatticePlanner::heuristic(const GridState & state, const GridState & goal) const
{
  const double distance = std::hypot(state.x - goal.x, state.y - goal.y) * map_->resolution;
  const int raw_yaw = std::abs(state.yaw - goal.yaw);
  return distance + 0.05 * std::min(raw_yaw, config_.yaw_bins - raw_yaw);
}

bool LatticePlanner::transition(
  const SearchState & current, const Motion & motion, const PlanningOverlay & overlay,
  SearchState & next, double & transition_cost) const
{
  next = current;
  if (motion.yaw_delta != 0) {
    next.grid.yaw =
      (current.grid.yaw + motion.yaw_delta + config_.yaw_bins) % config_.yaw_bins;
    transition_cost = config_.yaw_change_cost;
    return true;
  }
  const double yaw = yawAngle(current.grid.yaw);
  const double scale = config_.motion_step / map_->resolution;
  const double dx = (std::cos(yaw) * motion.forward -
    std::sin(yaw) * motion.lateral) * scale;
  const double dy = (std::sin(yaw) * motion.forward +
    std::cos(yaw) * motion.lateral) * scale;
  if (!std::isfinite(scale) || !std::isfinite(dx) || !std::isfinite(dy) ||
    std::abs(dx) > kMaximumIntegerSafeDistance ||
    std::abs(dy) > kMaximumIntegerSafeDistance)
  {
    return false;
  }
  const std::int64_t candidate_x =
    static_cast<std::int64_t>(current.grid.x) + std::llround(dx);
  const std::int64_t candidate_y =
    static_cast<std::int64_t>(current.grid.y) + std::llround(dy);
  if (candidate_x < 0 || candidate_y < 0 ||
    candidate_x >= static_cast<std::int64_t>(map_->width) ||
    candidate_y >= static_cast<std::int64_t>(map_->height))
  {
    return false;
  }
  next.grid.x = static_cast<int>(candidate_x);
  next.grid.y = static_cast<int>(candidate_y);
  if (next.grid.x == current.grid.x && next.grid.y == current.grid.y) {
    return false;
  }

  CellProperties current_properties;
  CellProperties next_properties;
  if (!cellProperties(
      current.grid.x, current.grid.y, overlay, current.inferred_prefix,
      current_properties))
  {
    return false;
  }
  if (observedValidCell(next.grid.x, next.grid.y)) {
    next.inferred_prefix = false;
  } else if (!current.inferred_prefix ||
    !cellProperties(next.grid.x, next.grid.y, overlay, true, next_properties))
  {
    return false;
  }
  if (!cellProperties(
      next.grid.x, next.grid.y, overlay, next.inferred_prefix, next_properties))
  {
    return false;
  }

  const double dz = next_properties.elevation - current_properties.elevation;
  if (std::abs(dz) > config_.max_step_height) {return false;}
  // A significant height jump is treated as a stair edge. Cross it longitudinally,
  // because sideways stepping provides a much smaller support margin.
  if (std::abs(dz) > config_.stair_height_threshold &&
    std::abs(motion.lateral) > 0.5 * std::abs(motion.forward)) {return false;}

  const double distance = std::hypot(dx, dy) * map_->resolution;
  transition_cost = distance * motion.factor +
    config_.terrain_cost_weight * (1.0 - next_properties.traversability) * distance +
    config_.slope_cost_weight * next_properties.slope * distance +
    config_.height_cost_weight * std::abs(dz);
  return true;
}

PlannedGridState LatticePlanner::plannedState(
  const SearchState & state, const PlanningOverlay & overlay) const
{
  CellProperties properties;
  if (!cellProperties(
      state.grid.x, state.grid.y, overlay, state.inferred_prefix, properties))
  {
    throw std::logic_error("search result references an invalid terrain cell");
  }

  PlannedGridState planned;
  planned.x = state.grid.x;
  planned.y = state.grid.y;
  planned.yaw = state.grid.yaw;
  planned.inferred = properties.inferred;
  planned.elevation = properties.elevation;
  if (properties.inferred) {
    planned.dzdx = overlay.plane_x;
    planned.dzdy = overlay.plane_y;
  } else {
    planned.dzdx =
      (surfaceElevation(planned.x + 1, planned.y, overlay, planned.elevation) -
      surfaceElevation(planned.x - 1, planned.y, overlay, planned.elevation)) /
      (2.0 * map_->resolution);
    planned.dzdy =
      (surfaceElevation(planned.x, planned.y + 1, overlay, planned.elevation) -
      surfaceElevation(planned.x, planned.y - 1, overlay, planned.elevation)) /
      (2.0 * map_->resolution);
  }
  return planned;
}

}  // namespace utree_dog_navigation
