#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "utree_go2_sdk2_bridge/simple_navigation_geometry.hpp"

namespace utree_go2_sdk2_bridge
{

struct SimpleNavigationConfig
{
  double position_tolerance{0.15};
  double yaw_tolerance{0.12};
  double align_tolerance{0.08};
  double waypoint_cross_track_tolerance{0.30};
  double linear_gain{1.0};
  double lateral_gain{1.0};
  double yaw_gain{1.5};
  double max_vx{0.6};
  double max_vy{0.35};
  double max_yaw_rate{0.8};
};

struct SimpleNavigationPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SimpleNavigationGoal
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum class SimpleNavigationPhase
{
  kIdle,
  kAlignSegment,
  kTranslateSegment,
};

struct SimpleNavigationCommand
{
  bool valid{false};
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  bool goal_reached{false};
  bool segment_aligned{false};
  std::size_t waypoints_reached{0};
  std::size_t route_index{0};
  SimpleNavigationPhase phase{SimpleNavigationPhase::kIdle};
};

// Pure right-angle navigation state machine. It has no ROS, DDS, or SDK
// dependency, so it can be exercised with deterministic kinematic simulation.
class SimpleNavigationController
{
public:
  SimpleNavigationController() = default;

  void setConfig(const SimpleNavigationConfig & config);
  void setGoal(const SimpleNavigationGoal & goal);
  void clearGoal();
  void prepareRoute(const SimpleNavigationPose & pose);

  bool hasGoal() const;
  bool hasRoute() const;
  std::size_t routeIndex() const;
  SimpleNavigationPhase phase() const;
  const std::vector<SimpleWaypoint> & route() const;

  SimpleNavigationCommand update(const SimpleNavigationPose & pose);

private:
  void rebuildRoute(const SimpleNavigationPose & pose);
  std::size_t advanceReachedSegments(const SimpleNavigationPose & pose);
  bool currentTargetReached(const SimpleNavigationPose & pose) const;
  std::optional<double> desiredSegmentYaw() const;
  static double normalizeAngle(double angle);
  static double clamp(double value, double limit);

  SimpleNavigationConfig config_{};
  std::optional<SimpleNavigationGoal> goal_;
  std::vector<SimpleWaypoint> route_;
  std::size_t route_index_{0};
  SimpleWaypoint segment_start_{};
  bool has_segment_start_{false};
  SimpleNavigationPhase phase_{SimpleNavigationPhase::kIdle};
};

}  // namespace utree_go2_sdk2_bridge
