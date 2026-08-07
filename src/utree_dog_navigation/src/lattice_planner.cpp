#include "utree_dog_navigation/lattice_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace utree_dog_navigation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDistanceTolerance = 1.0e-6;

double normalizeAngle(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle <= -kPi) {angle += 2.0 * kPi;}
  return angle;
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
}  // namespace

LatticePlanner::LatticePlanner(LatticePlannerConfig config) : config_(std::move(config))
{
  config_.yaw_bins = std::max(8, config_.yaw_bins);
  if (config_.start_snap_radius < 0.0) {
    config_.start_snap_radius = config_.snap_radius;
  }
}

void LatticePlanner::setMap(utree_dog_msgs::msg::TerrainGrid::SharedPtr map)
{
  map_ = std::move(map);
}

bool LatticePlanner::hasMap() const noexcept {return static_cast<bool>(map_);}

bool LatticePlanner::mapValid() const
{
  if (!map_ || map_->resolution <= 0.0F || map_->width == 0 || map_->height == 0) {
    return false;
  }
  const std::size_t expected = static_cast<std::size_t>(map_->width) * map_->height;
  return map_->elevation.size() == expected && map_->slope.size() == expected &&
         map_->traversability.size() == expected;
}

PlanningResult LatticePlanner::plan(const WorldState & start_world, const WorldState & goal_world) const
{
  PlanningResult result;
  if (!mapValid()) {return result;}

  GridState start;
  GridState goal;
  if (!toGrid(start_world.x, start_world.y, start.x, start.y) ||
    !toGrid(goal_world.x, goal_world.y, goal.x, goal.y)) {return result;}
  start.yaw = yawBin(start_world.yaw);
  goal.yaw = yawBin(goal_world.yaw);
  if (!nearestValid(
      start_world.x, start_world.y, config_.start_snap_radius, start.x, start.y) ||
    !nearestValid(goal_world.x, goal_world.y, config_.snap_radius, goal.x, goal.y))
  {
    return result;
  }

  const std::array<Motion, 10> motions{{
    {1.0, 0.0, 0, 1.0}, {-1.0, 0.0, 0, config_.reverse_cost_factor},
    {0.0, 1.0, 0, config_.lateral_cost_factor},
    {0.0, -1.0, 0, config_.lateral_cost_factor},
    {0.7071, 0.7071, 0, 1.1}, {0.7071, -0.7071, 0, 1.1},
    {-0.7071, 0.7071, 0, 1.3}, {-0.7071, -0.7071, 0, 1.3},
    {0.0, 0.0, 1, 1.0}, {0.0, 0.0, -1, 1.0}}};

  // Sparse records keep memory proportional to explored states rather than map size * yaw bins.
  std::priority_queue<QueueItem> open;
  std::unordered_map<std::uint64_t, SearchRecord> records;
  const std::uint64_t start_key = key(start);
  records[start_key].g = 0.0;
  open.push({heuristic(start, goal), 0.0, start_key});
  std::uint64_t reached_key = 0;

  while (!open.empty() && result.expansions < config_.max_expansions) {
    const QueueItem item = open.top();
    open.pop();
    auto current_record = records.find(item.key);
    if (current_record == records.end() || current_record->second.closed ||
      item.g > current_record->second.g) {continue;}
    current_record->second.closed = true;
    const double current_g = current_record->second.g;
    const GridState current = decode(item.key);
    ++result.expansions;
    if (current.x == goal.x && current.y == goal.y && current.yaw == goal.yaw) {
      reached_key = item.key;
      result.success = true;
      break;
    }

    for (const auto & motion : motions) {
      GridState next;
      double edge_cost = 0.0;
      if (!transition(current, motion, next, edge_cost)) {continue;}
      const std::uint64_t next_key = key(next);
      const double next_g = current_g + edge_cost;
      auto & next_record = records[next_key];
      if (next_record.closed || next_g >= next_record.g) {continue;}
      next_record.g = next_g;
      next_record.parent = item.key;
      next_record.has_parent = true;
      open.push({next_g + heuristic(next, goal), next_g, next_key});
    }
  }

  if (!result.success) {return result;}
  for (std::uint64_t cursor = reached_key;;) {
    result.states.push_back(decode(cursor));
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

bool LatticePlanner::toGrid(double x, double y, int & gx, int & gy) const
{
  gx = static_cast<int>(std::floor((x - map_->origin_x) / map_->resolution));
  gy = static_cast<int>(std::floor((y - map_->origin_y) / map_->resolution));
  return inside(gx, gy);
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

std::uint64_t LatticePlanner::key(const GridState & state) const
{
  return static_cast<std::uint64_t>(cellAddress(state.x, state.y)) * config_.yaw_bins + state.yaw;
}

GridState LatticePlanner::decode(std::uint64_t value) const
{
  GridState state;
  state.yaw = static_cast<int>(value % config_.yaw_bins);
  const std::uint64_t cell = value / config_.yaw_bins;
  state.x = static_cast<int>(cell % map_->width);
  state.y = static_cast<int>(cell / map_->width);
  return state;
}

int LatticePlanner::yawBin(double yaw) const
{
  const double positive = normalizeAngle(yaw) + kPi;
  int bin = static_cast<int>(std::lround(positive * config_.yaw_bins / (2.0 * kPi)));
  return (bin + config_.yaw_bins / 2) % config_.yaw_bins;
}

bool LatticePlanner::validCell(int x, int y) const
{
  if (!inside(x, y)) {return false;}
  const std::size_t i = cellAddress(x, y);
  return map_->traversability[i] != map_->unknown_value &&
         map_->traversability[i] >= config_.min_traversability &&
         map_->slope[i] != map_->unknown_value && map_->slope[i] <= config_.max_slope;
}

bool LatticePlanner::nearestValid(
  double world_x, double world_y, double snap_radius, int & x, int & y) const
{
  if (validCell(x, y)) {return true;}
  const int radius = std::max(
    1, static_cast<int>(std::ceil(snap_radius / map_->resolution)));
  int best_x = x;
  int best_y = y;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int candidate_x = x + dx;
      const int candidate_y = y + dy;
      if (!validCell(candidate_x, candidate_y)) {continue;}
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

double LatticePlanner::heuristic(const GridState & state, const GridState & goal) const
{
  const double distance = std::hypot(state.x - goal.x, state.y - goal.y) * map_->resolution;
  const int raw_yaw = std::abs(state.yaw - goal.yaw);
  return distance + 0.05 * std::min(raw_yaw, config_.yaw_bins - raw_yaw);
}

bool LatticePlanner::transition(
  const GridState & current, const Motion & motion, GridState & next,
  double & transition_cost) const
{
  next = current;
  if (motion.yaw_delta != 0) {
    next.yaw = (current.yaw + motion.yaw_delta + config_.yaw_bins) % config_.yaw_bins;
    transition_cost = config_.yaw_change_cost;
    return true;
  }
  const double yaw = yawAngle(current.yaw);
  const double scale = config_.motion_step / map_->resolution;
  const double dx = (std::cos(yaw) * motion.forward -
    std::sin(yaw) * motion.lateral) * scale;
  const double dy = (std::sin(yaw) * motion.forward +
    std::cos(yaw) * motion.lateral) * scale;
  next.x += static_cast<int>(std::lround(dx));
  next.y += static_cast<int>(std::lround(dy));
  if ((next.x == current.x && next.y == current.y) || !validCell(next.x, next.y)) {
    return false;
  }
  const std::size_t current_i = cellAddress(current.x, current.y);
  const std::size_t next_i = cellAddress(next.x, next.y);
  const double dz = map_->elevation[next_i] - map_->elevation[current_i];
  if (std::abs(dz) > config_.max_step_height) {return false;}
  // A significant height jump is treated as a stair edge. Cross it longitudinally,
  // because sideways stepping provides a much smaller support margin.
  if (std::abs(dz) > config_.stair_height_threshold &&
    std::abs(motion.lateral) > 0.5 * std::abs(motion.forward)) {return false;}

  const double distance = std::hypot(dx, dy) * map_->resolution;
  transition_cost = distance * motion.factor +
    config_.terrain_cost_weight * (1.0 - map_->traversability[next_i]) * distance +
    config_.slope_cost_weight * map_->slope[next_i] * distance +
    config_.height_cost_weight * std::abs(dz);
  return true;
}

}  // namespace utree_dog_navigation
