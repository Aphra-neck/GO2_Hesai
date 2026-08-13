#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>
#include <vector>

#include "utree_dog_msgs/msg/terrain_grid.hpp"

namespace utree_dog_navigation
{

struct VerifiedFlatStartConfig
{
  bool enabled{false};
  double support_inner_radius{1.0};
  double support_outer_radius{2.5};
  double fill_radius{1.35};
  int sector_count{8};
  int min_supported_sectors{7};
  int min_cells_per_sector{3};
  int min_support_cells{32};
  int min_observation_count{4};
  double max_plane_slope{0.15};
  double max_plane_rmse{0.04};
  double max_plane_residual{0.10};
  double max_elevation_range{0.18};
  double inferred_traversability{0.20};
};

enum class PlanningMode : std::uint8_t
{
  kTerrain,
  kFlatObstacle,
};

struct FlatObstacleConfig
{
  double footprint_length{0.90};
  double footprint_width{0.55};
  double obstacle_clearance{0.10};
  // Surface elevation used by PlannedGridState. The node may instead lock body Z directly.
  double surface_elevation{0.0};
};

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
  VerifiedFlatStartConfig verified_flat_start{};
  PlanningMode planning_mode{PlanningMode::kTerrain};
  FlatObstacleConfig flat_obstacle{};
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

enum class VerifiedFlatStartStatus : std::uint8_t
{
  kNotNeeded,
  kDisabled,
  kApplied,
  kInvalidConfiguration,
  kMissingObservationLayer,
  kInsufficientSupport,
  kInsufficientSectors,
  kPlaneFitFailed,
  kSupportNotFlat,
  kNoInferredStartCell,
  kNoObservedConnection,
};

std::string_view verifiedFlatStartStatusName(VerifiedFlatStartStatus status) noexcept;

enum class PlanningFailureReason : std::uint8_t
{
  kNone,
  kInvalidInput,
  kEndpointOutsideMap,
  kExactStartCollision,
  kStartGridSnapCollision,
  kGoalFootprintUnavailable,
  kStartTerrainUnavailable,
  kGoalTerrainUnavailable,
  kCancelled,
  kSearchExhausted,
  kExpansionLimit,
};

std::string_view planningFailureReasonName(PlanningFailureReason reason) noexcept;

struct PlannedGridState
{
  int x{0};
  int y{0};
  int yaw{0};
  bool inferred{false};
  double elevation{0.0};
  double dzdx{0.0};
  double dzdy{0.0};
};

struct PlanningResult
{
  bool success{false};
  int expansions{0};
  double path_cost{0.0};
  PlanningFailureReason failure_reason{PlanningFailureReason::kNone};
  VerifiedFlatStartStatus start_status{VerifiedFlatStartStatus::kNotNeeded};
  bool include_exact_start{false};
  bool exact_start_inferred{false};
  bool start_connector_translation{false};
  double exact_start_elevation{0.0};
  double exact_start_dzdx{0.0};
  double exact_start_dzdy{0.0};
  std::vector<PlannedGridState> states;
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
  PlanningResult plan(
    const WorldState & start, const WorldState & goal,
    const std::function<bool()> & cancellation_requested = {}) const;
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

  struct SearchState
  {
    GridState grid;
    bool inferred_prefix{false};
  };

  struct PlanningOverlay
  {
    bool active{false};
    double center_x{0.0};
    double center_y{0.0};
    double plane_x{0.0};
    double plane_y{0.0};
    double plane_z{0.0};
    double slope{0.0};
    double traversability{0.0};
    std::vector<std::uint8_t> inferred_cells;
  };

  struct CellProperties
  {
    double elevation{0.0};
    double slope{0.0};
    double traversability{0.0};
    bool inferred{false};
  };

  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  std::array<Motion, 10> motions() const;
  bool toGrid(double x, double y, int & gx, int & gy) const;
  bool inside(int x, int y) const;
  std::size_t cellAddress(int x, int y) const;
  std::uint64_t key(const SearchState & state) const;
  SearchState decode(std::uint64_t value) const;
  int yawBin(double yaw) const;
  bool observedValidCell(int x, int y) const;
  bool flatObstacleCell(int x, int y) const;
  bool flatPoseCollisionFree(int x, int y, int yaw, double padding = 0.0) const;
  bool flatWorldPoseCollisionFree(
    double world_x, double world_y, double yaw, double padding = 0.0) const;
  bool flatTransitionCollisionFree(
    const GridState & current, const GridState & next) const;
  bool flatTranslationCollisionFree(
    double start_x, double start_y, double end_x, double end_y, double yaw) const;
  bool flatRotationCollisionFree(
    double world_x, double world_y, double start_yaw, double end_yaw) const;
  bool flatPolygonCollisionFree(const std::vector<Point2D> & polygon) const;
  bool nearestReachableFlatStart(
    double world_x, double world_y, double world_yaw, double snap_radius,
    int yaw, int & x, int & y) const;
  bool nearestFlatValid(
    double world_x, double world_y, double snap_radius, int yaw, int & x, int & y) const;
  bool nearestObservedValid(
    double world_x, double world_y, double snap_radius, int & x, int & y) const;
  bool verifiedFlatConfigurationValid() const;
  VerifiedFlatStartStatus buildVerifiedFlatOverlay(
    double start_x, double start_y, PlanningOverlay & overlay) const;
  bool overlayCell(int x, int y, const PlanningOverlay & overlay) const;
  bool inferredStartConnectsToObserved(
    const SearchState & start, const PlanningOverlay & overlay) const;
  bool cellProperties(
    int x, int y, const PlanningOverlay & overlay, bool allow_inferred,
    CellProperties & properties) const;
  double surfaceElevation(
    int x, int y, const PlanningOverlay & overlay, double fallback) const;
  double heuristic(const GridState & state, const GridState & goal) const;
  bool transition(
    const SearchState & current, const Motion & motion, const PlanningOverlay & overlay,
    SearchState & next, double & transition_cost) const;
  PlannedGridState plannedState(
    const SearchState & state, const PlanningOverlay & overlay) const;

  LatticePlannerConfig config_;
  utree_dog_msgs::msg::TerrainGrid::SharedPtr map_;
};

}  // namespace utree_dog_navigation
