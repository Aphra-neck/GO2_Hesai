#include "utree_dog_navigation/body_lattice_planner_node.hpp"

#include <cmath>
#include <functional>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

namespace utree_dog_navigation
{

BodyLatticePlannerNode::BodyLatticePlannerNode() : Node("body_lattice_planner")
{
  map_topic_ = declare_parameter("terrain_map_topic", "terrain_map");
  odom_topic_ = declare_parameter("odom_topic", "/lio/body_odom");
  goal_topic_ = declare_parameter("goal_topic", "/goal_pose");
  path_topic_ = declare_parameter("path_topic", "body_path");
  nominal_body_height_ = declare_parameter("nominal_body_height", 0.42);
  LatticePlannerConfig config;
  config.yaw_bins = declare_parameter("yaw_bins", 16);
  config.motion_step = declare_parameter("motion_step", 0.20);
  config.min_traversability = declare_parameter("min_traversability", 0.18);
  config.max_step_height = declare_parameter("max_step_height", 0.24);
  config.max_slope = declare_parameter("max_slope", 0.65);
  config.stair_height_threshold = declare_parameter("stair_height_threshold", 0.08);
  config.terrain_cost_weight = declare_parameter("terrain_cost_weight", 4.0);
  config.slope_cost_weight = declare_parameter("slope_cost_weight", 1.5);
  config.height_cost_weight = declare_parameter("height_cost_weight", 2.0);
  config.yaw_change_cost = declare_parameter("yaw_change_cost", 0.15);
  config.reverse_cost_factor = declare_parameter("reverse_cost_factor", 1.15);
  config.lateral_cost_factor = declare_parameter("lateral_cost_factor", 1.25);
  config.max_expansions = declare_parameter("max_expansions", 250000);
  config.snap_radius = declare_parameter("snap_radius", 0.5);
  config.start_snap_radius = declare_parameter("start_snap_radius", config.snap_radius);
  planner_ = std::make_unique<LatticePlanner>(config);

  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  map_sub_ = create_subscription<utree_dog_msgs::msg::TerrainGrid>(
    map_topic_, map_qos,
    std::bind(&BodyLatticePlannerNode::mapCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&BodyLatticePlannerNode::odomCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(10),
    std::bind(&BodyLatticePlannerNode::goalCallback, this, std::placeholders::_1));
  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, map_qos);
}

void BodyLatticePlannerNode::mapCallback(
  const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg)
{
  planner_->setMap(msg);
  if (!planner_->mapValid()) {
    RCLCPP_ERROR(get_logger(), "Rejected malformed terrain map");
    return;
  }
  if (have_odom_ && have_goal_) {requestPlan();}
}

void BodyLatticePlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_ = msg;
  have_odom_ = true;
}

void BodyLatticePlannerNode::goalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_ = msg;
  have_goal_ = true;
  requestPlan();
}

void BodyLatticePlannerNode::requestPlan()
{
  if (!planner_->hasMap() || !have_odom_ || !have_goal_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for map, odometry and goal");
    return;
  }
  const auto & map = planner_->map();
  if (goal_->header.frame_id != map.header.frame_id) {
    RCLCPP_ERROR(
      get_logger(), "Goal frame '%s' must match terrain frame '%s'",
      goal_->header.frame_id.c_str(), map.header.frame_id.c_str());
    return;
  }
  const WorldState start{
    odom_->pose.pose.position.x, odom_->pose.pose.position.y,
    tf2::getYaw(odom_->pose.pose.orientation)};
  const WorldState goal{
    goal_->pose.position.x, goal_->pose.position.y, tf2::getYaw(goal_->pose.orientation)};
  const PlanningResult result = planner_->plan(start, goal);
  if (!result.success) {
    RCLCPP_ERROR(get_logger(), "Planning failed after %d state expansions", result.expansions);
    return;
  }
  path_pub_->publish(makePath(result.states));
  RCLCPP_INFO(
    get_logger(), "Planned %zu body poses after %d expansions",
    result.states.size(), result.expansions);
}

nav_msgs::msg::Path BodyLatticePlannerNode::makePath(
  const std::vector<GridState> & states) const
{
  const auto & map = planner_->map();
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = map.header.frame_id;
  path.poses.reserve(states.size());
  for (const auto & state : states) {
    const std::size_t index = static_cast<std::size_t>(state.y) * map.width + state.x;
    const double z = map.elevation[index];
    const double dzdx =
      (planner_->elevationAt(state.x + 1, state.y, z) -
      planner_->elevationAt(state.x - 1, state.y, z)) / (2.0 * map.resolution);
    const double dzdy =
      (planner_->elevationAt(state.x, state.y + 1, z) -
      planner_->elevationAt(state.x, state.y - 1, z)) / (2.0 * map.resolution);

    // Body z is measured above the support surface; roll/pitch follow its local normal.
    tf2::Quaternion orientation;
    orientation.setRPY(
      std::atan2(dzdy, std::sqrt(1.0 + dzdx * dzdx)),
      -std::atan2(dzdx, 1.0), planner_->yawAngle(state.yaw));
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = map.origin_x + (state.x + 0.5) * map.resolution;
    pose.pose.position.y = map.origin_y + (state.y + 0.5) * map.resolution;
    pose.pose.position.z = z + nominal_body_height_;
    pose.pose.orientation = tf2::toMsg(orientation);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace utree_dog_navigation
