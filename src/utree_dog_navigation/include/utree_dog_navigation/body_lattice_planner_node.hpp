#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utree_dog_msgs/msg/terrain_grid.hpp"
#include "utree_dog_navigation/lattice_planner.hpp"

namespace utree_dog_navigation
{

// ROS adapter that translates odometry/goals to lattice states and publishes body poses.
class BodyLatticePlannerNode : public rclcpp::Node
{
public:
  explicit BodyLatticePlannerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void mapCallback(const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void requestPlan();
  void watchdogTick();
  bool expireGoalIfNeeded();
  bool goalRetentionExpired() const;
  bool framesValid();
  bool posesValid();
  bool inputsFresh(const rclcpp::Time & current_time);
  bool stampFresh(
    const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & current_time,
    double maximum_age, const char * input_name);
  void clearForStaleInput(const char * reason);
  void clearPath(const char * reason);
  void clearGoal(const char * reason);
  nav_msgs::msg::Path makePath(
    const PlanningResult & result, const WorldState & exact_start,
    const builtin_interfaces::msg::Time & source_stamp,
    const builtin_interfaces::msg::Time & goal_stamp) const;

  std::string map_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string map_frame_;
  std::string body_frame_;
  double nominal_body_height_{0.42};
  double max_map_age_{1.0};
  double max_odom_age_{0.5};
  double max_goal_age_{2.0};
  double goal_retention_timeout_{30.0};
  double timestamp_future_tolerance_{0.2};
  double input_watchdog_rate_{10.0};
  PlanningMode planning_mode_{PlanningMode::kTerrain};
  bool flat_ground_confirmed_{false};
  bool flat_body_height_locked_{false};
  double flat_body_height_z_{0.0};
  bool have_odom_{false};
  bool have_goal_{false};
  bool path_active_{false};
  std::string last_path_frame_;
  std::unique_ptr<LatticePlanner> planner_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  std::chrono::steady_clock::time_point goal_received_time_{};
  rclcpp::Subscription<utree_dog_msgs::msg::TerrainGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr input_watchdog_timer_;
};

}  // namespace utree_dog_navigation
