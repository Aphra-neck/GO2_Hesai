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

std::string_view planningFailureReasonName(PlanningFailureReason reason) noexcept
{
  switch (reason) {
    case PlanningFailureReason::kNone: return "none";
    case PlanningFailureReason::kInvalidInput: return "invalid_input";
    case PlanningFailureReason::kEndpointOutsideMap: return "endpoint_outside_map";
    case PlanningFailureReason::kExactStartCollision: return "exact_start_collision";
    case PlanningFailureReason::kStartGridSnapCollision: return "start_grid_snap_collision";
    case PlanningFailureReason::kGoalFootprintUnavailable: return "goal_footprint_unavailable";
    case PlanningFailureReason::kStartTerrainUnavailable: return "start_terrain_unavailable";
    case PlanningFailureReason::kGoalTerrainUnavailable: return "goal_terrain_unavailable";
    case PlanningFailureReason::kCancelled: return "cancelled";
    case PlanningFailureReason::kSearchExhausted: return "search_exhausted";
    case PlanningFailureReason::kExpansionLimit: return "expansion_limit";
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
  if (config_.planning_mode == PlanningMode::kFlatObstacle) {
    const auto & flat = config_.flat_obstacle;
    const bool valid_flat_config =
      std::isfinite(flat.footprint_length) && flat.footprint_length > 0.0 &&
      std::isfinite(flat.footprint_width) && flat.footprint_width > 0.0 &&
      std::isfinite(flat.obstacle_clearance) && flat.obstacle_clearance >= 0.0 &&
      std::isfinite(flat.surface_elevation);
    if (!valid_flat_config) {
      throw std::invalid_argument("flat obstacle planner configuration is invalid");
    }
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
  if (map_->traversability.size() != expected)
  {
    return false;
  }
  if (config_.planning_mode == PlanningMode::kTerrain &&
    (map_->elevation.size() != expected || map_->slope.size() != expected))
  {
    return false;
  }
  for (std::size_t index = 0; index < expected; ++index) {
    const float traversability = map_->traversability[index];
    if (!std::isfinite(traversability)) {return false;}
    if (traversability != map_->unknown_value &&
      (traversability < 0.0F || traversability > 1.0F))
    {
      return false;
    }
    if (config_.planning_mode == PlanningMode::kFlatObstacle) {continue;}
    const float elevation = map_->elevation[index];
    const float slope = map_->slope[index];
    if (!std::isfinite(elevation) || !std::isfinite(slope)) {return false;}
    if (slope != map_->unknown_value && slope < 0.0F) {return false;}
  }
  return true;
}

PlanningResult LatticePlanner::plan(
  const WorldState & start_world, const WorldState & goal_world,
  const std::function<bool()> & cancellation_requested) const
{
  PlanningResult result;
  if (!mapValid() || !finiteWorldState(start_world) || !finiteWorldState(goal_world)) {
    result.failure_reason = PlanningFailureReason::kInvalidInput;
    return result;
  }

  SearchState start;
  GridState goal;
  if (!toGrid(start_world.x, start_world.y, start.grid.x, start.grid.y) ||
    !toGrid(goal_world.x, goal_world.y, goal.x, goal.y))
  {
    result.failure_reason = PlanningFailureReason::kEndpointOutsideMap;
    return result;
  }
  start.grid.yaw = yawBin(start_world.yaw);
  goal.yaw = yawBin(goal_world.yaw);

  PlanningOverlay overlay;
  if (config_.planning_mode == PlanningMode::kFlatObstacle) {
    const bool exact_start_collision = !flatWorldPoseCollisionFree(
      start_world.x, start_world.y, start_world.yaw);
    if (exact_start_collision) {
      if (!config_.flat_obstacle.recover_colliding_start ||
        !nearestFlatValid(
          start_world.x, start_world.y, config_.start_snap_radius,
          start.grid.yaw, start.grid.x, start.grid.y))
      {
        result.failure_reason = PlanningFailureReason::kExactStartCollision;
        return result;
      }
      result.colliding_start_recovered = true;
    } else if (!nearestReachableFlatStart(
        start_world.x, start_world.y, start_world.yaw, config_.start_snap_radius,
        start.grid.yaw, start.grid.x, start.grid.y))
    {
      result.failure_reason = PlanningFailureReason::kStartGridSnapCollision;
      return result;
    }
    if (!nearestFlatValid(
        goal_world.x, goal_world.y, config_.snap_radius, goal.yaw, goal.x, goal.y))
    {
      result.failure_reason = PlanningFailureReason::kGoalFootprintUnavailable;
      return result;
    }
    result.include_exact_start = true;
    const double grid_start_x =
      map_->origin_x + (static_cast<double>(start.grid.x) + 0.5) * map_->resolution;
    const double grid_start_y =
      map_->origin_y + (static_cast<double>(start.grid.y) + 0.5) * map_->resolution;
    result.start_connector_translation =
      std::hypot(grid_start_x - start_world.x, grid_start_y - start_world.y) >
      kDistanceTolerance;
    result.exact_start_elevation = config_.flat_obstacle.surface_elevation;
  } else {
    if (!nearestObservedValid(
        start_world.x, start_world.y, config_.start_snap_radius,
        start.grid.x, start.grid.y))
    {
      if (!config_.verified_flat_start.enabled) {
        result.start_status = VerifiedFlatStartStatus::kDisabled;
        result.failure_reason = PlanningFailureReason::kStartTerrainUnavailable;
        return result;
      }
      result.start_status = buildVerifiedFlatOverlay(start_world.x, start_world.y, overlay);
      if (result.start_status != VerifiedFlatStartStatus::kApplied) {
        result.failure_reason = PlanningFailureReason::kStartTerrainUnavailable;
        return result;
      }
      if (!overlayCell(start.grid.x, start.grid.y, overlay)) {
        result.start_status = VerifiedFlatStartStatus::kNoInferredStartCell;
        result.failure_reason = PlanningFailureReason::kStartTerrainUnavailable;
        return result;
      }
      start.inferred_prefix = true;
      if (!inferredStartConnectsToObserved(start, overlay)) {
        result.start_status = VerifiedFlatStartStatus::kNoObservedConnection;
        result.failure_reason = PlanningFailureReason::kStartTerrainUnavailable;
        return result;
      }
      result.exact_start_inferred = true;
      result.exact_start_elevation = overlay.plane_z;
      result.exact_start_dzdx = overlay.plane_x;
      result.exact_start_dzdy = overlay.plane_y;
    }
    if (!nearestObservedValid(
        goal_world.x, goal_world.y, config_.snap_radius, goal.x, goal.y))
    {
      result.failure_reason = PlanningFailureReason::kGoalTerrainUnavailable;
      return result;
    }
  }

  const auto planner_motions = motions();

  // Sparse records keep memory proportional to explored states rather than map size * yaw bins.
  std::priority_queue<QueueItem> open;
  std::unordered_map<std::uint64_t, SearchRecord> records;
  const std::uint64_t start_key = key(start);
  records[start_key].g = 0.0;
  open.push({heuristic(start.grid, goal), 0.0, start_key});
  std::uint64_t reached_key = 0;

  while (!open.empty() && result.expansions < config_.max_expansions) {
    if (cancellation_requested && cancellation_requested()) {
      result.failure_reason = PlanningFailureReason::kCancelled;
      return result;
    }
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
      result.path_cost = current_g;
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

  if (!result.success) {
    result.failure_reason = result.expansions >= config_.max_expansions && !open.empty() ?
      PlanningFailureReason::kExpansionLimit : PlanningFailureReason::kSearchExhausted;
    return result;
  }
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
  if (config_.planning_mode == PlanningMode::kFlatObstacle) {
    return inside(x, y) ? config_.flat_obstacle.surface_elevation : fallback;
  }
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

bool LatticePlanner::flatObstacleCell(int x, int y) const
{
  if (!inside(x, y)) {return true;}
  const float traversability = map_->traversability[cellAddress(x, y)];
  return traversability != map_->unknown_value && traversability <= 0.0F;
}

bool LatticePlanner::flatPoseCollisionFree(int x, int y, int yaw, double padding) const
{
  if (!inside(x, y)) {return false;}
  const double world_x = map_->origin_x + (static_cast<double>(x) + 0.5) * map_->resolution;
  const double world_y = map_->origin_y + (static_cast<double>(y) + 0.5) * map_->resolution;
  return flatWorldPoseCollisionFree(world_x, world_y, yawAngle(yaw), padding);
}

bool LatticePlanner::flatWorldPoseCollisionFree(
  double world_x, double world_y, double yaw, double padding) const
{
  const double half_length = 0.5 * config_.flat_obstacle.footprint_length +
    config_.flat_obstacle.obstacle_clearance + padding;
  const double half_width = 0.5 * config_.flat_obstacle.footprint_width +
    config_.flat_obstacle.obstacle_clearance + padding;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  std::vector<Point2D> polygon;
  polygon.reserve(4);
  for (const auto & local : std::array<Point2D, 4>{{
      {-half_length, -half_width}, {half_length, -half_width},
      {half_length, half_width}, {-half_length, half_width}}})
  {
    polygon.push_back({
      world_x + cosine * local.x - sine * local.y,
      world_y + sine * local.x + cosine * local.y});
  }
  return flatPolygonCollisionFree(polygon);
}

bool LatticePlanner::flatPolygonCollisionFree(const std::vector<Point2D> & polygon) const
{
  if (polygon.size() < 3) {return false;}

  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : polygon) {
    minimum_x = std::min(minimum_x, point.x);
    maximum_x = std::max(maximum_x, point.x);
    minimum_y = std::min(minimum_y, point.y);
    maximum_y = std::max(maximum_y, point.y);
  }

  const double map_minimum_x = map_->origin_x;
  const double map_minimum_y = map_->origin_y;
  const double map_maximum_x = map_->origin_x + map_->width * map_->resolution;
  const double map_maximum_y = map_->origin_y + map_->height * map_->resolution;
  if (minimum_x < map_minimum_x - kDistanceTolerance ||
    minimum_y < map_minimum_y - kDistanceTolerance ||
    maximum_x > map_maximum_x + kDistanceTolerance ||
    maximum_y > map_maximum_y + kDistanceTolerance)
  {
    return false;
  }

  const int first_x = std::clamp(
    static_cast<int>(std::floor((minimum_x - map_->origin_x) / map_->resolution)),
    0, static_cast<int>(map_->width) - 1);
  const int last_x = std::clamp(
    static_cast<int>(std::floor((maximum_x - map_->origin_x) / map_->resolution)),
    0, static_cast<int>(map_->width) - 1);
  const int first_y = std::clamp(
    static_cast<int>(std::floor((minimum_y - map_->origin_y) / map_->resolution)),
    0, static_cast<int>(map_->height) - 1);
  const int last_y = std::clamp(
    static_cast<int>(std::floor((maximum_y - map_->origin_y) / map_->resolution)),
    0, static_cast<int>(map_->height) - 1);

  const auto separated_on_axis = [&polygon](
    const Point2D & axis, double cell_minimum_x, double cell_maximum_x,
    double cell_minimum_y, double cell_maximum_y)
    {
      if (std::abs(axis.x) + std::abs(axis.y) < kDistanceTolerance) {return false;}
      double polygon_minimum = std::numeric_limits<double>::infinity();
      double polygon_maximum = -std::numeric_limits<double>::infinity();
      for (const auto & point : polygon) {
        const double projection = axis.x * point.x + axis.y * point.y;
        polygon_minimum = std::min(polygon_minimum, projection);
        polygon_maximum = std::max(polygon_maximum, projection);
      }
      const std::array<Point2D, 4> cell{{
        {cell_minimum_x, cell_minimum_y}, {cell_maximum_x, cell_minimum_y},
        {cell_maximum_x, cell_maximum_y}, {cell_minimum_x, cell_maximum_y}}};
      double cell_minimum = std::numeric_limits<double>::infinity();
      double cell_maximum = -std::numeric_limits<double>::infinity();
      for (const auto & point : cell) {
        const double projection = axis.x * point.x + axis.y * point.y;
        cell_minimum = std::min(cell_minimum, projection);
        cell_maximum = std::max(cell_maximum, projection);
      }
      return polygon_maximum < cell_minimum - kDistanceTolerance ||
             cell_maximum < polygon_minimum - kDistanceTolerance;
    };

  for (int y = first_y; y <= last_y; ++y) {
    for (int x = first_x; x <= last_x; ++x) {
      if (!flatObstacleCell(x, y)) {continue;}
      const double cell_minimum_x = map_->origin_x + x * map_->resolution;
      const double cell_minimum_y = map_->origin_y + y * map_->resolution;
      const double cell_maximum_x = cell_minimum_x + map_->resolution;
      const double cell_maximum_y = cell_minimum_y + map_->resolution;
      bool separated = separated_on_axis(
        {1.0, 0.0}, cell_minimum_x, cell_maximum_x, cell_minimum_y, cell_maximum_y) ||
        separated_on_axis(
        {0.0, 1.0}, cell_minimum_x, cell_maximum_x, cell_minimum_y, cell_maximum_y);
      for (std::size_t index = 0; !separated && index < polygon.size(); ++index) {
        const Point2D & first = polygon[index];
        const Point2D & second = polygon[(index + 1) % polygon.size()];
        separated = separated_on_axis(
          {-(second.y - first.y), second.x - first.x},
          cell_minimum_x, cell_maximum_x, cell_minimum_y, cell_maximum_y);
      }
      if (!separated) {return false;}
    }
  }
  return true;
}

bool LatticePlanner::flatTransitionCollisionFree(
  const GridState & current, const GridState & next) const
{
  const double current_x =
    map_->origin_x + (static_cast<double>(current.x) + 0.5) * map_->resolution;
  const double current_y =
    map_->origin_y + (static_cast<double>(current.y) + 0.5) * map_->resolution;
  const double next_x =
    map_->origin_x + (static_cast<double>(next.x) + 0.5) * map_->resolution;
  const double next_y =
    map_->origin_y + (static_cast<double>(next.y) + 0.5) * map_->resolution;
  const double current_yaw = yawAngle(current.yaw);
  const double next_yaw = yawAngle(next.yaw);

  if (current.x != next.x || current.y != next.y) {
    return flatTranslationCollisionFree(current_x, current_y, next_x, next_y, current_yaw);
  }

  return flatRotationCollisionFree(current_x, current_y, current_yaw, next_yaw);
}

bool LatticePlanner::flatTranslationCollisionFree(
  double start_x, double start_y, double end_x, double end_y, double yaw) const
{
  const double half_length = 0.5 * config_.flat_obstacle.footprint_length +
    config_.flat_obstacle.obstacle_clearance;
  const double half_width = 0.5 * config_.flat_obstacle.footprint_width +
    config_.flat_obstacle.obstacle_clearance;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  std::vector<Point2D> points;
  points.reserve(8);
  for (const auto & center : std::array<Point2D, 2>{{
      {start_x, start_y}, {end_x, end_y}}})
  {
    for (const auto & local : std::array<Point2D, 4>{{
        {-half_length, -half_width}, {half_length, -half_width},
        {half_length, half_width}, {-half_length, half_width}}})
    {
      points.push_back({
        center.x + cosine * local.x - sine * local.y,
        center.y + sine * local.x + cosine * local.y});
    }
  }

  std::sort(points.begin(), points.end(), [](const Point2D & lhs, const Point2D & rhs) {
    return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
  });
  const auto cross = [](const Point2D & origin, const Point2D & first, const Point2D & second) {
    return (first.x - origin.x) * (second.y - origin.y) -
           (first.y - origin.y) * (second.x - origin.x);
  };
  std::vector<Point2D> hull;
  hull.reserve(points.size() * 2);
  for (const auto & point : points) {
    while (hull.size() >= 2 &&
      cross(hull[hull.size() - 2], hull.back(), point) <= kDistanceTolerance)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (auto iterator = points.rbegin() + 1; iterator != points.rend(); ++iterator) {
    while (hull.size() > lower_size &&
      cross(hull[hull.size() - 2], hull.back(), *iterator) <= kDistanceTolerance)
    {
      hull.pop_back();
    }
    hull.push_back(*iterator);
  }
  if (!hull.empty()) {hull.pop_back();}
  return flatPolygonCollisionFree(hull);
}

bool LatticePlanner::flatRotationCollisionFree(
  double world_x, double world_y, double start_yaw, double end_yaw) const
{
  const double yaw_delta = normalizeAngle(end_yaw - start_yaw);
  const double radius = std::hypot(
    0.5 * config_.flat_obstacle.footprint_length + config_.flat_obstacle.obstacle_clearance,
    0.5 * config_.flat_obstacle.footprint_width + config_.flat_obstacle.obstacle_clearance);
  const double sample_spacing = std::max(
    0.01, std::min(0.05, 0.5 * static_cast<double>(map_->resolution)));
  const int intervals = std::max(
    1, static_cast<int>(std::ceil(radius * std::abs(yaw_delta) / sample_spacing)));
  // Padding contains the corner arc between adjacent samples, making this conservative.
  const double padding = radius * std::abs(yaw_delta) / (2.0 * intervals);
  for (int sample = 0; sample <= intervals; ++sample) {
    const double fraction = static_cast<double>(sample) / intervals;
    if (!flatWorldPoseCollisionFree(
        world_x, world_y, start_yaw + fraction * yaw_delta, padding))
    {
      return false;
    }
  }
  return true;
}

bool LatticePlanner::nearestReachableFlatStart(
  double world_x, double world_y, double world_yaw, double snap_radius,
  int yaw, int & x, int & y) const
{
  const double snapped_yaw = yawAngle(yaw);
  const auto connector_collision_free = [this, world_x, world_y, world_yaw, snapped_yaw](
      int candidate_x, int candidate_y)
    {
      const double candidate_world_x =
        map_->origin_x + (static_cast<double>(candidate_x) + 0.5) * map_->resolution;
      const double candidate_world_y =
        map_->origin_y + (static_cast<double>(candidate_y) + 0.5) * map_->resolution;
      return flatTranslationCollisionFree(
        world_x, world_y, candidate_world_x, candidate_world_y, world_yaw) &&
             flatRotationCollisionFree(
        candidate_world_x, candidate_world_y, world_yaw, snapped_yaw);
    };

  // Preserve the containing-cell behavior whenever its executable connector is safe.
  if (connector_collision_free(x, y)) {return true;}

  const double radius_cells = std::ceil(snap_radius / map_->resolution);
  const int map_max_x = static_cast<int>(map_->width) - 1;
  const int map_max_y = static_cast<int>(map_->height) - 1;
  const int radius = std::max(1, static_cast<int>(std::min(
      radius_cells, static_cast<double>(std::max(map_->width, map_->height)))));
  const int minimum_x = static_cast<int>(std::max<std::int64_t>(
      0, static_cast<std::int64_t>(x) - radius));
  const int maximum_x = static_cast<int>(std::min<std::int64_t>(
      map_max_x, static_cast<std::int64_t>(x) + radius));
  const int minimum_y = static_cast<int>(std::max<std::int64_t>(
      0, static_cast<std::int64_t>(y) - radius));
  const int maximum_y = static_cast<int>(std::min<std::int64_t>(
      map_max_y, static_cast<std::int64_t>(y) + radius));

  int best_x = x;
  int best_y = y;
  double best_distance = std::numeric_limits<double>::infinity();
  // Ascending y/x scan order is the stable tie-break for equally near candidates.
  for (int candidate_y = minimum_y; candidate_y <= maximum_y; ++candidate_y) {
    for (int candidate_x = minimum_x; candidate_x <= maximum_x; ++candidate_x) {
      const double candidate_world_x =
        map_->origin_x + (static_cast<double>(candidate_x) + 0.5) * map_->resolution;
      const double candidate_world_y =
        map_->origin_y + (static_cast<double>(candidate_y) + 0.5) * map_->resolution;
      const double distance = std::hypot(
        candidate_world_x - world_x, candidate_world_y - world_y);
      if (distance > snap_radius + kDistanceTolerance ||
        distance >= best_distance - kDistanceTolerance ||
        !connector_collision_free(candidate_x, candidate_y))
      {
        continue;
      }
      best_x = candidate_x;
      best_y = candidate_y;
      best_distance = distance;
    }
  }
  if (!std::isfinite(best_distance)) {return false;}
  x = best_x;
  y = best_y;
  return true;
}

bool LatticePlanner::nearestFlatValid(
  double world_x, double world_y, double snap_radius, int yaw, int & x, int & y) const
{
  if (flatPoseCollisionFree(x, y, yaw)) {return true;}
  const double radius_cells = std::ceil(snap_radius / map_->resolution);
  const int map_max_x = static_cast<int>(map_->width) - 1;
  const int map_max_y = static_cast<int>(map_->height) - 1;
  const int radius = std::max(1, static_cast<int>(std::min(
      radius_cells, static_cast<double>(std::max(map_->width, map_->height)))));
  const int minimum_x = static_cast<int>(std::max<std::int64_t>(
      0, static_cast<std::int64_t>(x) - radius));
  const int maximum_x = static_cast<int>(std::min<std::int64_t>(
      map_max_x, static_cast<std::int64_t>(x) + radius));
  const int minimum_y = static_cast<int>(std::max<std::int64_t>(
      0, static_cast<std::int64_t>(y) - radius));
  const int maximum_y = static_cast<int>(std::min<std::int64_t>(
      map_max_y, static_cast<std::int64_t>(y) + radius));
  int best_x = x;
  int best_y = y;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int candidate_y = minimum_y; candidate_y <= maximum_y; ++candidate_y) {
    for (int candidate_x = minimum_x; candidate_x <= maximum_x; ++candidate_x) {
      if (!flatPoseCollisionFree(candidate_x, candidate_y, yaw)) {continue;}
      const double candidate_world_x =
        map_->origin_x + (static_cast<double>(candidate_x) + 0.5) * map_->resolution;
      const double candidate_world_y =
        map_->origin_y + (static_cast<double>(candidate_y) + 0.5) * map_->resolution;
      const double distance = std::hypot(
        candidate_world_x - world_x, candidate_world_y - world_y);
      if (distance > snap_radius + kDistanceTolerance || distance >= best_distance) {continue;}
      best_x = candidate_x;
      best_y = candidate_y;
      best_distance = distance;
    }
  }
  if (!std::isfinite(best_distance)) {return false;}
  x = best_x;
  y = best_y;
  return true;
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
  if (config_.planning_mode == PlanningMode::kFlatObstacle) {
    if (flatObstacleCell(x, y)) {return false;}
    properties.elevation = config_.flat_obstacle.surface_elevation;
    properties.slope = 0.0;
    properties.traversability = 1.0;
    properties.inferred = false;
    return true;
  }
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
  if (config_.planning_mode == PlanningMode::kFlatObstacle) {
    return inside(x, y) ? config_.flat_obstacle.surface_elevation : fallback;
  }
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
  const int yaw_distance = std::min(raw_yaw, config_.yaw_bins - raw_yaw);
  if (config_.planning_mode == PlanningMode::kFlatObstacle) {
    const double minimum_translation_factor = std::min({
      1.0, config_.reverse_cost_factor, config_.lateral_cost_factor, 1.1, 1.3});
    return distance * minimum_translation_factor + config_.yaw_change_cost * yaw_distance;
  }
  return distance + 0.05 * yaw_distance;
}

bool LatticePlanner::transition(
  const SearchState & current, const Motion & motion, const PlanningOverlay & overlay,
  SearchState & next, double & transition_cost) const
{
  next = current;
  if (motion.yaw_delta != 0) {
    next.grid.yaw =
      (current.grid.yaw + motion.yaw_delta + config_.yaw_bins) % config_.yaw_bins;
    if (config_.planning_mode == PlanningMode::kFlatObstacle &&
      !flatTransitionCollisionFree(current.grid, next.grid))
    {
      return false;
    }
    transition_cost = config_.yaw_change_cost;
    return true;
  }
  const double yaw = yawAngle(current.grid.yaw);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double scale = config_.motion_step / map_->resolution;
  const double dx = (cosine * motion.forward - sine * motion.lateral) * scale;
  const double dy = (sine * motion.forward + cosine * motion.lateral) * scale;
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

  if (config_.planning_mode == PlanningMode::kFlatObstacle) {
    if (!flatTransitionCollisionFree(current.grid, next.grid)) {return false;}
    next.inferred_prefix = false;
    const double grid_dx = static_cast<double>(next.grid.x - current.grid.x);
    const double grid_dy = static_cast<double>(next.grid.y - current.grid.y);
    const double grid_distance = std::hypot(grid_dx, grid_dy);
    if (!std::isfinite(grid_distance) || grid_distance <= kDistanceTolerance) {
      return false;
    }
    const double forward_alignment = std::clamp(
      (cosine * grid_dx + sine * grid_dy) / grid_distance, -1.0, 1.0);
    // Penalize sustained motion away from the body's forward tangent. The
    // chord metric is cheap, accounts for grid rounding, and leaves short
    // omnidirectional corrections available when turning would cost more.
    const double heading_misalignment_cost = config_.yaw_change_cost *
      static_cast<double>(config_.yaw_bins) * 0.25 * (1.0 - forward_alignment);
    transition_cost = grid_distance * map_->resolution * motion.factor +
      heading_misalignment_cost;
    return true;
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
  if (config_.planning_mode == PlanningMode::kFlatObstacle) {
    planned.dzdx = 0.0;
    planned.dzdy = 0.0;
  } else if (properties.inferred) {
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
