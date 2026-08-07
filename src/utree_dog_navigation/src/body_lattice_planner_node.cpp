#include "utree_dog_navigation/body_lattice_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

namespace utree_dog_navigation
{
namespace
{
constexpr double kQuaternionNormEpsilon = 1.0e-12;
constexpr double kQuaternionNormTolerance = 1.0e-3;

bool validPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & position = pose.position;
  const auto & orientation = pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
    !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
    !std::isfinite(orientation.w))
  {
    return false;
  }
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return std::isfinite(norm_squared) && norm_squared > kQuaternionNormEpsilon &&
         std::abs(norm_squared - 1.0) <= kQuaternionNormTolerance;
}

bool validRosTimestamp(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U &&
         (stamp.sec != 0 || stamp.nanosec != 0U);
}
}  // namespace

BodyLatticePlannerNode::BodyLatticePlannerNode(const rclcpp::NodeOptions & options)
: Node("body_lattice_planner", options)
{
  map_topic_ = declare_parameter("terrain_map_topic", "terrain_map");
  odom_topic_ = declare_parameter("odom_topic", "/lio/body_odom");
  goal_topic_ = declare_parameter("goal_topic", "/goal_pose");
  path_topic_ = declare_parameter("path_topic", "body_path");
  map_frame_ = declare_parameter("map_frame", "world");
  body_frame_ = declare_parameter("body_frame", "base_link");
  nominal_body_height_ = declare_parameter("nominal_body_height", 0.42);
  max_map_age_ = declare_parameter("max_map_age", 1.0);
  max_odom_age_ = declare_parameter("max_odom_age", 0.5);
  timestamp_future_tolerance_ = declare_parameter("timestamp_future_tolerance", 0.2);
  input_watchdog_rate_ = declare_parameter("input_watchdog_rate", 10.0);
  const auto finite_in_range = [](double value, double minimum, double maximum) {
      return std::isfinite(value) && value >= minimum && value <= maximum;
    };
  if (!finite_in_range(max_map_age_, 0.001, 60.0)) {
    throw std::invalid_argument("max_map_age must be finite and in [0.001, 60] seconds");
  }
  if (!finite_in_range(max_odom_age_, 0.001, 60.0)) {
    throw std::invalid_argument("max_odom_age must be finite and in [0.001, 60] seconds");
  }
  if (!finite_in_range(timestamp_future_tolerance_, 0.0, 5.0)) {
    throw std::invalid_argument(
            "timestamp_future_tolerance must be finite and in [0, 5] seconds");
  }
  if (!finite_in_range(input_watchdog_rate_, 0.1, 100.0)) {
    throw std::invalid_argument("input_watchdog_rate must be finite and in [0.1, 100] Hz");
  }
  if (map_frame_.empty() || body_frame_.empty()) {
    throw std::invalid_argument("map_frame and body_frame must not be empty");
  }
  const double watchdog_period_seconds = 1.0 / input_watchdog_rate_;
  if (watchdog_period_seconds > std::min(max_map_age_, max_odom_age_)) {
    throw std::invalid_argument(
            "input watchdog period must not exceed max_map_age or max_odom_age");
  }
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
  const auto watchdog_period = std::chrono::duration<double>(1.0 / input_watchdog_rate_);
  input_watchdog_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(watchdog_period),
    std::bind(&BodyLatticePlannerNode::watchdogTick, this));
}

void BodyLatticePlannerNode::mapCallback(
  const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg)
{
  planner_->setMap(msg);
  if (!planner_->mapValid()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Rejected malformed terrain map");
    clearPath("malformed terrain map");
    return;
  }
  if (msg->header.frame_id != map_frame_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected terrain frame '%s'; expected '%s'",
      msg->header.frame_id.c_str(), map_frame_.c_str());
    clearPath("terrain frame does not match configured map frame");
    return;
  }
  if (have_odom_ && have_goal_) {requestPlan();}
}

void BodyLatticePlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_ = msg;
  have_odom_ = true;
  if (odom_->header.frame_id != map_frame_ || odom_->child_frame_id != body_frame_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected body odometry frames '%s' -> '%s'; expected '%s' -> '%s'",
      odom_->header.frame_id.c_str(), odom_->child_frame_id.c_str(),
      map_frame_.c_str(), body_frame_.c_str());
    clearPath("body odometry frame contract changed");
    return;
  }
  if (!validPose(odom_->pose.pose)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected body odometry with a non-finite position or invalid quaternion");
    clearPath("body odometry pose is malformed");
    return;
  }
  if (path_active_ && !stampFresh(
      odom_->header.stamp, now(), max_odom_age_, "body odometry"))
  {
    clearPath("body odometry timestamp became stale or invalid");
  }
}

void BodyLatticePlannerNode::goalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_ = msg;
  have_goal_ = true;
  if (!validPose(goal_->pose)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected goal with a non-finite position or invalid quaternion");
    clearPath("goal pose is malformed");
    return;
  }
  requestPlan();
}

void BodyLatticePlannerNode::requestPlan()
{
  if (!planner_->hasMap() || !have_odom_ || !have_goal_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for map, odometry and goal");
    clearPath("required planning input is unavailable");
    return;
  }
  if (!framesValid()) {
    clearPath("planning frame contract is invalid");
    return;
  }
  if (!posesValid()) {
    clearPath("planning pose input is malformed");
    return;
  }
  const auto & map = planner_->map();
  const rclcpp::Time current_time = now();
  if (!inputsFresh(current_time)) {
    clearPath("stale or invalid map/odometry timestamp");
    return;
  }
  const WorldState start{
    odom_->pose.pose.position.x, odom_->pose.pose.position.y,
    tf2::getYaw(odom_->pose.pose.orientation)};
  const WorldState goal{
    goal_->pose.position.x, goal_->pose.position.y, tf2::getYaw(goal_->pose.orientation)};
  const PlanningResult result = planner_->plan(
    start, goal, [this]() {return !inputsFresh(now());});
  const rclcpp::Time completion_time = now();
  if (!framesValid() || !posesValid() || !inputsFresh(completion_time)) {
    clearPath("planning inputs became stale or invalid during search");
    return;
  }
  if (!result.success) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Planning failed after %d state expansions", result.expansions);
    clearPath("planning failed");
    return;
  }
  const rclcpp::Time map_time(map.header.stamp, completion_time.get_clock_type());
  const rclcpp::Time odom_time(odom_->header.stamp, completion_time.get_clock_type());
  const auto & source_stamp = map_time <= odom_time ? map.header.stamp : odom_->header.stamp;
  const nav_msgs::msg::Path path = makePath(result.states, source_stamp);
  const rclcpp::Time publish_time = now();
  if (!framesValid() || !posesValid() || !inputsFresh(publish_time)) {
    clearPath("planning inputs became stale or invalid before path publication");
    return;
  }
  path_pub_->publish(path);
  path_active_ = true;
  last_path_frame_ = map.header.frame_id;
  RCLCPP_INFO(
    get_logger(), "Planned %zu body poses after %d expansions",
    result.states.size(), result.expansions);
}

void BodyLatticePlannerNode::watchdogTick()
{
  if (!path_active_) {return;}
  if (!planner_->hasMap() || !have_odom_ || !have_goal_ ||
    !framesValid() || !posesValid() || !inputsFresh(now()))
  {
    clearPath("planning inputs stopped or became stale");
  }
}

bool BodyLatticePlannerNode::framesValid()
{
  if (!planner_->hasMap() || !odom_ || !goal_) {return false;}
  const auto & map = planner_->map();
  const bool valid = map.header.frame_id == map_frame_ &&
    odom_->header.frame_id == map_frame_ && odom_->child_frame_id == body_frame_ &&
    goal_->header.frame_id == map_frame_;
  if (!valid) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Planning frames must be map='%s', odom='%s' -> '%s', goal='%s'; "
      "received map='%s', odom='%s' -> '%s', goal='%s'",
      map_frame_.c_str(), map_frame_.c_str(), body_frame_.c_str(), map_frame_.c_str(),
      map.header.frame_id.c_str(), odom_->header.frame_id.c_str(),
      odom_->child_frame_id.c_str(), goal_->header.frame_id.c_str());
  }
  return valid;
}

bool BodyLatticePlannerNode::posesValid()
{
  if (!odom_ || !goal_) {return false;}
  const bool valid = validPose(odom_->pose.pose) && validPose(goal_->pose);
  if (!valid) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Planning odometry and goal must have finite positions and unit quaternions");
  }
  return valid;
}

bool BodyLatticePlannerNode::inputsFresh(const rclcpp::Time & current_time)
{
  return stampFresh(
    planner_->map().header.stamp, current_time, max_map_age_, "terrain map") &&
         stampFresh(
    odom_->header.stamp, current_time, max_odom_age_, "body odometry");
}

bool BodyLatticePlannerNode::stampFresh(
  const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & current_time,
  double maximum_age, const char * input_name)
{
  if (!validRosTimestamp(stamp)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected %s with invalid timestamp sec=%d nanosec=%u",
      input_name, stamp.sec, stamp.nanosec);
    return false;
  }
  const rclcpp::Time message_time(stamp, current_time.get_clock_type());
  const double age = (current_time - message_time).seconds();
  if (message_time.nanoseconds() <= 0 || !std::isfinite(age) ||
    age < -timestamp_future_tolerance_ || age > maximum_age)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected %s with age %.3f s; accepted range is [-%.3f, %.3f] s",
      input_name, age, timestamp_future_tolerance_, maximum_age);
    return false;
  }
  return true;
}

void BodyLatticePlannerNode::clearPath(const char * reason)
{
  if (!path_active_) {return;}
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = last_path_frame_;
  path_pub_->publish(path);
  path_active_ = false;
  RCLCPP_WARN(get_logger(), "Cleared body path: %s", reason);
}

nav_msgs::msg::Path BodyLatticePlannerNode::makePath(
  const std::vector<GridState> & states,
  const builtin_interfaces::msg::Time & source_stamp) const
{
  const auto & map = planner_->map();
  nav_msgs::msg::Path path;
  path.header.stamp = source_stamp;
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
