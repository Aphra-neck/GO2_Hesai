#pragma once

#include <vector>

namespace utree_go2_sdk2_bridge
{

struct SimpleWaypoint
{
  double x;
  double y;
};

// Builds a Manhattan route with at most one right-angle waypoint. The first
// leg is selected by the current heading so the robot aligns before translating.
std::vector<SimpleWaypoint> makeRightAngleRoute(
  double start_x, double start_y, double start_yaw,
  double goal_x, double goal_y, double position_tolerance);

SimpleWaypoint targetDeltaInBody(
  double current_x, double current_y, double current_yaw,
  double target_x, double target_y);

}  // namespace utree_go2_sdk2_bridge
