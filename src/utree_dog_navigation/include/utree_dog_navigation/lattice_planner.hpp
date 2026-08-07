#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "utree_dog_msgs/msg/terrain_grid.hpp"

namespace utree_dog_navigation
{

struct LatticePlannerConfig
{
  int yaw_bins{16};
  double motion_step{0.20};
  double min_traversability{0.18};
  double max_step_height{0.24};
  double max_slope{0.65};
  double stair_height_threshold{0.08};
  double terrain_cost_weight{4.0};
  double slope_cost_weight{1.5};
  double height_cost_weight{2.0};
  double yaw_change_cost{0.15};
  double reverse_cost_factor{1.15};
  double lateral_cost_factor{1.25};
  int max_expansions{250000};
  double snap_radius{0.5};
  // A negative value preserves the legacy contract: use snap_radius for both endpoints.
  double start_snap_radius{-1.0};
};

struct GridState
{
  int x{0};
  int y{0};
  int yaw{0};
};

struct WorldState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PlanningResult
{
  bool success{false};
  int expansions{0};
  std::vector<GridState> states;
};

// Sparse A* over body position and heading. The class is independent of ROS nodes
// and can be tested with a constructed TerrainGrid message.
class LatticePlanner
{
public:
  explicit LatticePlanner(LatticePlannerConfig config);

  void setMap(utree_dog_msgs::msg::TerrainGrid::SharedPtr map);
  bool hasMap() const noexcept;
  bool mapValid() const;
  PlanningResult plan(const WorldState & start, const WorldState & goal) const;
  double yawAngle(int bin) const;
  double elevationAt(int x, int y, double fallback) const;
  const utree_dog_msgs::msg::TerrainGrid & map() const;

private:
  struct Motion
  {
    double forward;
    double lateral;
    int yaw_delta;
    double factor;
  };

  bool toGrid(double x, double y, int & gx, int & gy) const;
  bool inside(int x, int y) const;
  std::size_t cellAddress(int x, int y) const;
  std::uint64_t key(const GridState & state) const;
  GridState decode(std::uint64_t value) const;
  int yawBin(double yaw) const;
  bool validCell(int x, int y) const;
  bool nearestValid(
    double world_x, double world_y, double snap_radius, int & x, int & y) const;
  double heuristic(const GridState & state, const GridState & goal) const;
  bool transition(
    const GridState & current, const Motion & motion, GridState & next,
    double & transition_cost) const;

  LatticePlannerConfig config_;
  utree_dog_msgs::msg::TerrainGrid::SharedPtr map_;
};

}  // namespace utree_dog_navigation
