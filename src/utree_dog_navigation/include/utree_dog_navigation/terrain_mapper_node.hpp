#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "utree_dog_msgs/msg/terrain_grid.hpp"
#include "utree_dog_navigation/terrain_map_builder.hpp"

namespace utree_dog_navigation
{

// ROS adapter for TerrainMapBuilder. It owns topic I/O and sensor-frame filtering.
class TerrainMapperNode : public rclcpp::Node
{
public:
  explicit TerrainMapperNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishMap();
  nav_msgs::msg::OccupancyGrid makeCostmap(
    const utree_dog_msgs::msg::TerrainGrid & terrain) const;

  std::string map_frame_;
  std::string cloud_topic_;
  std::string odom_topic_;
  double min_range_{0.35};
  double max_range_{12.0};
  double min_z_relative_{-1.5};
  double max_z_relative_{1.5};
  double self_length_{0.9};
  double self_width_{0.55};
  double self_height_{0.7};
  double integration_window_{1.5};
  double confidence_rebuild_start_radius_{0.55};
  int min_observed_frames_{4};
  double publish_rate_{2.0};
  double cloud_stale_warning_age_{1.0};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_z_{0.0};
  double robot_yaw_{0.0};
  bool have_odom_{false};
  bool have_cloud_interval_{false};
  bool last_published_start_feature_ready_{false};
  bool confidence_rebuild_active_{false};
  std::size_t suppressed_confidence_rebuild_maps_{0};
  double last_cloud_source_delta_{0.0};
  double last_cloud_receive_gap_{0.0};
  std::size_t last_accepted_point_count_{0};
  std::size_t last_in_map_cell_count_{0};
  std::chrono::steady_clock::time_point last_cloud_received_{};
  builtin_interfaces::msg::Time last_cloud_stamp_;
  std::unique_ptr<TerrainMapBuilder> map_builder_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<utree_dog_msgs::msg::TerrainGrid>::SharedPtr terrain_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace utree_dog_navigation
