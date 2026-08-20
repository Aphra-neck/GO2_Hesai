#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "utree_dog_msgs/msg/terrain_grid.hpp"
#include "utree_dog_navigation/flat_obstacle_layer.hpp"
#include "utree_dog_navigation/terrain_map_builder.hpp"

namespace utree_dog_navigation
{

// ROS adapter for terrain and flat-obstacle mapping. It owns topic I/O and filtering.
class TerrainMapperNode : public rclcpp::Node
{
public:
  explicit TerrainMapperNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct FlatBodyPose
  {
    builtin_interfaces::msg::Time stamp;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{1.0};
  };

  struct PendingFlatCloud
  {
    sensor_msgs::msg::PointCloud2::SharedPtr message;
    std::chrono::steady_clock::time_point received_at;
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void processCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    const FlatBodyPose * flat_pose,
    std::chrono::steady_clock::time_point received_at);
  void cacheFlatPose(const FlatBodyPose & pose);
  const FlatBodyPose * findFlatPose(
    const builtin_interfaces::msg::Time & stamp) const;
  void queueFlatCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    std::chrono::steady_clock::time_point received_at);
  void processPendingFlatClouds(const FlatBodyPose & pose);
  void prunePendingFlatClouds(std::chrono::steady_clock::time_point now);
  bool flatPoseContinuous(
    const FlatBodyPose & pose, double & translation, double & angle) const;
  void invalidateFlatEpoch(const FlatBodyPose & pose);
  void publishMap();
  void publishUnusableFlatState(
    const builtin_interfaces::msg::Time & stamp,
    bool publish_current_filtered_points = false);
  utree_dog_msgs::msg::TerrainGrid makeFlatTerrain(
    const FlatObstacleLayerSnapshot & snapshot,
    const builtin_interfaces::msg::Time & stamp) const;
  nav_msgs::msg::OccupancyGrid makeCostmap(
    const utree_dog_msgs::msg::TerrainGrid & terrain,
    const std::vector<std::uint8_t> * inflated_obstacles = nullptr) const;
  nav_msgs::msg::GridCells makeFlatCells(
    const std::vector<std::uint8_t> & mask,
    const FlatObstacleLayerSnapshot & snapshot,
    const builtin_interfaces::msg::Time & stamp, double z_offset) const;
  nav_msgs::msg::OccupancyGrid makeUnknownFlatCostmap(
    const FlatObstacleLayerSnapshot & snapshot,
    const builtin_interfaces::msg::Time & stamp) const;
  sensor_msgs::msg::PointCloud2 makePointCloud(
    const std::vector<TerrainPoint> & points,
    const builtin_interfaces::msg::Time & stamp, bool downsample) const;
  TerrainPoint flatSensorOrigin(const FlatBodyPose & pose) const;

  std::string map_frame_;
  std::string body_frame_{"base_link"};
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string planning_mode_{"flat_obstacle"};
  bool flat_obstacle_mode_{false};
  double min_range_{0.35};
  double max_range_{12.0};
  double min_z_relative_{-1.5};
  double max_z_relative_{1.5};
  double self_length_{0.9};
  double self_width_{0.55};
  double self_height_{0.7};
  double integration_window_{1.5};
  double confidence_rebuild_start_radius_{0.80};
  int min_observed_frames_{4};
  double publish_rate_{2.0};
  double cloud_stale_warning_age_{1.0};
  double flat_max_odom_age_{0.5};
  double body_yaw_offset_{-1.5707963267948966};
  double lidar_offset_x_{0.171};
  double lidar_offset_y_{0.0};
  double lidar_offset_z_{0.0908};
  double visualization_voxel_size_{0.30};
  std::size_t visualization_max_points_{5000U};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_z_{0.0};
  double robot_yaw_{0.0};
  bool have_odom_{false};
  bool have_processed_flat_pose_{false};
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
  FlatBodyPose last_processed_flat_pose_;
  std::deque<FlatBodyPose> flat_odom_history_;
  std::deque<PendingFlatCloud> pending_flat_clouds_;
  std::unique_ptr<TerrainMapBuilder> map_builder_;
  std::unique_ptr<FlatObstacleLayer> flat_obstacle_layer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<utree_dog_msgs::msg::TerrainGrid>::SharedPtr terrain_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_pub_;
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr flat_raw_pub_;
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr flat_inflated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr flat_filtered_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr flat_filtered_map_3d_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace utree_dog_navigation
