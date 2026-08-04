#pragma once

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
  BodyLatticePlannerNode();

private:
  void mapCallback(const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void requestPlan();
  nav_msgs::msg::Path makePath(const std::vector<GridState> & states) const;

  std::string map_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  double nominal_body_height_{0.42};
  bool have_odom_{false};
  bool have_goal_{false};
  std::unique_ptr<LatticePlanner> planner_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  rclcpp::Subscription<utree_dog_msgs::msg::TerrainGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace utree_dog_navigation
