#include "utree_go2_sdk2_bridge/simple_navigation_geometry.hpp"

#include <cmath>

namespace utree_go2_sdk2_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double headingCost(double start_yaw, double x, double y)
{
  return std::abs(normalizeAngle(std::atan2(y, x) - start_yaw));
}

}  // namespace

std::vector<SimpleWaypoint> makeRightAngleRoute(
  double start_x, double start_y, double start_yaw,
  double goal_x, double goal_y, double position_tolerance)
{
  if (!std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(start_yaw) || !std::isfinite(goal_x) ||
    !std::isfinite(goal_y) || !std::isfinite(position_tolerance) ||
    position_tolerance < 0.0)
  {
    return {};
  }

  const double dx = goal_x - start_x;
  const double dy = goal_y - start_y;
  if (std::hypot(dx, dy) <= position_tolerance) {
    return {};
  }
  if (std::abs(dx) <= position_tolerance || std::abs(dy) <= position_tolerance) {
    return {{goal_x, goal_y}};
  }

  const SimpleWaypoint x_then_y{goal_x, start_y};
  const SimpleWaypoint y_then_x{start_x, goal_y};
  const double x_leg_cost = headingCost(start_yaw, dx, 0.0);
  const double y_leg_cost = headingCost(start_yaw, 0.0, dy);
  if (x_leg_cost <= y_leg_cost) {
    return {x_then_y, {goal_x, goal_y}};
  }
  return {y_then_x, {goal_x, goal_y}};
}

}  // namespace utree_go2_sdk2_bridge
