#include "utree_dog_navigation/terrain_mapper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace utree_dog_navigation
{
namespace
{
constexpr double kQuaternionNormEpsilon = 1.0e-12;

bool validRosTimestamp(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U &&
         (stamp.sec != 0 || stamp.nanosec != 0U);
}
}  // namespace

TerrainMapperNode::TerrainMapperNode(const rclcpp::NodeOptions & options)
: Node("terrain_mapper", options)
{
  map_frame_ = declare_parameter("map_frame", "world");
  body_frame_ = declare_parameter("body_frame", "base_link");
  cloud_topic_ = declare_parameter("cloud_topic", "/lio/cloud_world");
  odom_topic_ = declare_parameter("odom_topic", "/lio/body_odom");
  planning_mode_ = declare_parameter("planning_mode", "terrain");
  const bool flat_ground_confirmed = declare_parameter("flat_ground_confirmed", false);
  if (planning_mode_ != "terrain" && planning_mode_ != "flat_obstacle") {
    throw std::invalid_argument("planning_mode must be 'terrain' or 'flat_obstacle'");
  }
  flat_obstacle_mode_ = planning_mode_ == "flat_obstacle";
  if (flat_obstacle_mode_ && !flat_ground_confirmed) {
    throw std::invalid_argument(
            "flat_obstacle mode requires flat_ground_confirmed=true after the robot is standing");
  }
  TerrainMapConfig config;
  config.resolution = declare_parameter("resolution", 0.20);
  config.size_x = declare_parameter("size_x", 40.0);
  config.size_y = declare_parameter("size_y", 40.0);
  config.origin_x = declare_parameter("origin_x", -20.0);
  config.origin_y = declare_parameter("origin_y", -20.0);
  config.min_points_per_cell = declare_parameter("min_points_per_cell", 3);
  config.max_slope = declare_parameter("max_slope", 0.65);
  config.max_roughness = declare_parameter("max_roughness", 0.08);
  config.max_step_height = declare_parameter("max_step_height", 0.24);
  config.obstacle_height = declare_parameter("obstacle_height", 0.18);
  config.integration_window = declare_parameter("integration_window", 1.5);
  config.min_observed_frames = declare_parameter("min_observed_frames", 4);
  config.height_bin_resolution = declare_parameter("height_bin_resolution", 0.015);
  config.confidence_frames = declare_parameter("confidence_frames", 8.0);
  min_range_ = declare_parameter("min_range", 0.35);
  max_range_ = declare_parameter("max_range", 12.0);
  min_z_relative_ = declare_parameter("min_z_relative", -1.5);
  max_z_relative_ = declare_parameter("max_z_relative", 1.5);
  self_length_ = declare_parameter("self_filter.length", 0.9);
  self_width_ = declare_parameter("self_filter.width", 0.55);
  self_height_ = declare_parameter("self_filter.height", 0.7);
  integration_window_ = config.integration_window;
  min_observed_frames_ = config.min_observed_frames;
  confidence_rebuild_start_radius_ =
    declare_parameter("confidence_rebuild.start_radius", 0.55);
  if (!std::isfinite(integration_window_) || integration_window_ <= 0.0) {
    throw std::invalid_argument("integration_window must be finite and greater than zero");
  }
  if (min_observed_frames_ < 1 ||
    min_observed_frames_ > static_cast<int>(std::numeric_limits<std::uint16_t>::max()))
  {
    throw std::invalid_argument("min_observed_frames must be in [1, 65535]");
  }
  if (!std::isfinite(confidence_rebuild_start_radius_) ||
    confidence_rebuild_start_radius_ <= 0.0)
  {
    throw std::invalid_argument(
            "confidence_rebuild.start_radius must be finite and greater than zero");
  }
  publish_rate_ = declare_parameter("publish_rate", 2.0);
  cloud_stale_warning_age_ = declare_parameter("cloud_stale_warning_age", 1.0);
  if (!std::isfinite(cloud_stale_warning_age_) ||
    cloud_stale_warning_age_ < 0.001 || cloud_stale_warning_age_ > 60.0)
  {
    throw std::invalid_argument(
            "cloud_stale_warning_age must be finite and in [0.001, 60] seconds");
  }
  FlatObstacleLayerConfig flat_config;
  flat_config.min_height = declare_parameter("flat_obstacle.min_height", 0.08);
  flat_config.max_height = declare_parameter("flat_obstacle.max_height", 0.80);
  flat_config.clear_after = declare_parameter("flat_obstacle.clear_after", 1.0);
  flat_config.clearance = declare_parameter("flat_obstacle.obstacle_clearance", 0.10);
  flat_nominal_body_height_ =
    declare_parameter("flat_obstacle.nominal_body_height", 0.42);
  flat_max_odom_age_ = declare_parameter("flat_obstacle.max_odom_age", 0.5);
  if (!std::isfinite(flat_nominal_body_height_) || flat_nominal_body_height_ <= 0.0 ||
    flat_nominal_body_height_ > 2.0)
  {
    throw std::invalid_argument(
            "flat_obstacle.nominal_body_height must be finite and in (0, 2] metres");
  }
  if (!std::isfinite(flat_max_odom_age_) || flat_max_odom_age_ < 0.001 ||
    flat_max_odom_age_ > 60.0)
  {
    throw std::invalid_argument(
            "flat_obstacle.max_odom_age must be finite and in [0.001, 60] seconds");
  }
  if (flat_obstacle_mode_) {
    flat_map_builder_ = std::make_unique<FlatObstacleMapBuilder>(config, flat_config);
  } else {
    map_builder_ = std::make_unique<TerrainMapBuilder>(config);
  }

  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  terrain_pub_ = create_publisher<utree_dog_msgs::msg::TerrainGrid>("terrain_map", map_qos);
  cost_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("terrain_costmap", map_qos);
  if (flat_obstacle_mode_) {
    flat_raw_pub_ = create_publisher<nav_msgs::msg::GridCells>("flat_obstacle_raw", map_qos);
    flat_inflated_pub_ =
      create_publisher<nav_msgs::msg::GridCells>("flat_obstacle_inflated", map_qos);
  }
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TerrainMapperNode::cloudCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TerrainMapperNode::odomCallback, this, std::placeholders::_1));
  const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TerrainMapperNode::publishMap, this));

  RCLCPP_INFO(
    get_logger(), "%s map: %.1f x %.1f m, %.3f m/cell, %zu x %zu cells",
    flat_obstacle_mode_ ? "Flat obstacle" : "Terrain",
    config.size_x, config.size_y, config.resolution,
    flat_obstacle_mode_ ? flat_map_builder_->width() : map_builder_->width(),
    flat_obstacle_mode_ ? flat_map_builder_->height() : map_builder_->height());
}

void TerrainMapperNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & orientation = msg->pose.pose.orientation;
  const auto & position = msg->pose.pose.position;
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(norm_squared) || norm_squared <= kQuaternionNormEpsilon ||
    !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected odometry with a non-finite or zero-norm orientation");
    return;
  }
  if (flat_obstacle_mode_) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || msg->header.frame_id != map_frame_ ||
      msg->child_frame_id != body_frame_ || !validRosTimestamp(msg->header.stamp))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected flat-ground lock odometry with invalid position, frame, or timestamp");
      return;
    }
    const rclcpp::Time current_time = now();
    const rclcpp::Time odom_time(msg->header.stamp, current_time.get_clock_type());
    const double odom_age = (current_time - odom_time).seconds();
    if (!std::isfinite(odom_age) || odom_age > flat_max_odom_age_ || odom_age < -0.2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected flat-ground lock odometry with age %.3f s; accepted range is [-0.200, %.3f] s",
        odom_age, flat_max_odom_age_);
      return;
    }
  }

  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  const double x = orientation.x * inverse_norm;
  const double y = orientation.y * inverse_norm;
  const double z = orientation.z * inverse_norm;
  const double w = orientation.w * inverse_norm;
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_z_ = msg->pose.pose.position.z;
  robot_yaw_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  have_odom_ = true;
  if (flat_obstacle_mode_ && !flat_ground_locked_) {
    flat_ground_z_ = robot_z_ - flat_nominal_body_height_;
    flat_ground_locked_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Flat obstacle ground locked at z=%.3f m from standing body z=%.3f m; "
      "keep the robot standing while this mode is active",
      flat_ground_z_, robot_z_);
  }
}

void TerrainMapperNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for odometry");
    return;
  }
  // Super-LIO already publishes world-frame points. Reject mismatched frames instead of
  // silently applying a stale or unavailable transform.
  if (msg->header.frame_id != map_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Point cloud frame '%s' differs from map_frame '%s'; no TF is applied",
      msg->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  if (!validRosTimestamp(msg->header.stamp)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected point cloud with invalid timestamp sec=%d nanosec=%u",
      msg->header.stamp.sec, msg->header.stamp.nanosec);
    return;
  }
  if (flat_obstacle_mode_ && !flat_ground_locked_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Waiting to lock the flat ground plane from standing body odometry");
    return;
  }

  if (flat_obstacle_mode_) {
    const rclcpp::Time current_time = now();
    const rclcpp::Time cloud_time(msg->header.stamp, current_time.get_clock_type());
    const double header_age = (current_time - cloud_time).seconds();
    if (!std::isfinite(header_age) || header_age > cloud_stale_warning_age_ ||
      header_age < -0.2)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Freezing flat obstacle layer for stale cloud: header_age=%.3f s, threshold=%.3f s",
        header_age, cloud_stale_warning_age_);
      return;
    }
  }

  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
  std::vector<TerrainPoint> accepted_points;
  accepted_points.reserve(msg->width * msg->height / 2);
  const double cos_yaw = std::cos(robot_yaw_);
  const double sin_yaw = std::sin(robot_yaw_);
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
    const double dx = *x - robot_x_;
    const double dy = *y - robot_y_;
    const double dz = *z - robot_z_;
    const double range = std::hypot(dx, dy);
    if (range < min_range_ || range > max_range_ ||
      dz < min_z_relative_ || dz > max_z_relative_) {continue;}
    const double body_x = cos_yaw * dx + sin_yaw * dy;
    const double body_y = -sin_yaw * dx + cos_yaw * dy;
    if (std::abs(body_x) < self_length_ * 0.5 &&
      std::abs(body_y) < self_width_ * 0.5 &&
      std::abs(dz) < self_height_ * 0.5) {continue;}
    accepted_points.push_back({*x, *y, *z});
  }
  const auto received_at = std::chrono::steady_clock::now();
  const double stamp_seconds = static_cast<double>(msg->header.stamp.sec) +
    static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
  if (last_cloud_received_ != std::chrono::steady_clock::time_point{} &&
    validRosTimestamp(last_cloud_stamp_))
  {
    const double previous_stamp_seconds = static_cast<double>(last_cloud_stamp_.sec) +
      static_cast<double>(last_cloud_stamp_.nanosec) * 1.0e-9;
    last_cloud_source_delta_ = stamp_seconds - previous_stamp_seconds;
    last_cloud_receive_gap_ =
      std::chrono::duration<double>(received_at - last_cloud_received_).count();
    have_cloud_interval_ = true;
    if (!std::isfinite(last_cloud_source_delta_) || last_cloud_source_delta_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected nonmonotonic terrain source cloud timestamp: source_delta=%.3f s, "
        "receive_gap=%.3f s",
        last_cloud_source_delta_, last_cloud_receive_gap_);
      return;
    }
    const bool exceeds_integration_window =
      last_cloud_source_delta_ > integration_window_;
    if (exceeds_integration_window) {
      RCLCPP_WARN(
        get_logger(),
        "Terrain source cloud timestamp discontinuity: source_delta=%.3f s, "
        "receive_gap=%.3f s, integration_window=%.3f s",
        last_cloud_source_delta_, last_cloud_receive_gap_, integration_window_);
    }
  }
  bool accumulate_flat_misses = false;
  if (flat_obstacle_mode_ && have_cloud_interval_) {
    accumulate_flat_misses =
      last_cloud_source_delta_ <= cloud_stale_warning_age_ &&
      last_cloud_receive_gap_ <= cloud_stale_warning_age_;
  }
  if (flat_obstacle_mode_) {
    last_in_map_cell_count_ = flat_map_builder_->integrateFrame(
      accepted_points, stamp_seconds, flat_ground_z_, accumulate_flat_misses);
  } else {
    last_in_map_cell_count_ = map_builder_->integrateFrame(accepted_points, stamp_seconds);
  }
  last_accepted_point_count_ = accepted_points.size();
  last_cloud_stamp_ = msg->header.stamp;
  last_cloud_received_ = received_at;
}

void TerrainMapperNode::publishMap()
{
  const rclcpp::Time current_time = now();
  bool source_is_fresh = true;
  if (last_cloud_received_ == std::chrono::steady_clock::time_point{}) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Terrain map has not received an accepted world-frame point cloud");
    return;
  } else if (!validRosTimestamp(last_cloud_stamp_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Terrain source cloud state has an invalid timestamp sec=%d nanosec=%u",
      last_cloud_stamp_.sec, last_cloud_stamp_.nanosec);
    return;
  } else {
    const rclcpp::Time cloud_time(last_cloud_stamp_, current_time.get_clock_type());
    const double header_age = (current_time - cloud_time).seconds();
    const double receive_gap = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_cloud_received_).count();
    if (!std::isfinite(header_age) || !std::isfinite(receive_gap) ||
      header_age > cloud_stale_warning_age_ || receive_gap > cloud_stale_warning_age_ ||
      header_age < -0.2)
    {
      source_is_fresh = false;
      const char * classification = "invalid_timing";
      if (receive_gap > cloud_stale_warning_age_) {
        classification = "accepted_cloud_callbacks_stopped";
      } else if (header_age > cloud_stale_warning_age_) {
        classification = "stale_source_stamp_or_cloud_backlog";
      } else if (header_age < -0.2) {
        classification = "source_stamp_from_future";
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Terrain source cloud timing is invalid: classification=%s, header_age=%.3f s, "
        "receive_gap=%.3f s, warning threshold=%.3f s",
        classification, header_age, receive_gap, cloud_stale_warning_age_);
    }
  }
  if (flat_obstacle_mode_ && !source_is_fresh) {
    return;
  }
  const auto terrain = flat_obstacle_mode_ ?
    flat_map_builder_->build(last_cloud_stamp_, map_frame_, flat_ground_z_) :
    map_builder_->build(last_cloud_stamp_, map_frame_);
  if (flat_obstacle_mode_) {
    terrain_pub_->publish(terrain);
    cost_pub_->publish(makeCostmap(terrain));
    const auto raw = flat_map_builder_->rawObstacleMask();
    const auto inflated = flat_map_builder_->inflatedObstacleMask();
    flat_inflated_pub_->publish(makeFlatCells(inflated, last_cloud_stamp_, 0.01));
    flat_raw_pub_->publish(makeFlatCells(raw, last_cloud_stamp_, 0.02));
    return;
  }
  std::size_t observed_cells = 0;
  std::size_t observation_ready_cells = 0;
  std::size_t start_observation_ready_cells = 0;
  std::size_t feature_ready_cells = 0;
  std::size_t start_feature_ready_cells = 0;
  std::uint16_t maximum_observation_count = 0U;
  const auto minimum_observation_count = static_cast<std::size_t>(min_observed_frames_);
  const auto layer_known = [&terrain](float value) {
      return std::isfinite(value) && value != terrain.unknown_value;
    };
  for (std::size_t index = 0; index < terrain.observation_count.size(); ++index) {
    const auto count = terrain.observation_count[index];
    const std::size_t grid_x = index % terrain.width;
    const std::size_t grid_y = index / terrain.width;
    const double world_x =
      terrain.origin_x + (static_cast<double>(grid_x) + 0.5) * terrain.resolution;
    const double world_y =
      terrain.origin_y + (static_cast<double>(grid_y) + 0.5) * terrain.resolution;
    const bool in_start_radius =
      std::hypot(world_x - robot_x_, world_y - robot_y_) <=
      confidence_rebuild_start_radius_;
    if (count > 0U) {++observed_cells;}
    if (static_cast<std::size_t>(count) >= minimum_observation_count) {
      ++observation_ready_cells;
      if (in_start_radius) {++start_observation_ready_cells;}
    }
    if (layer_known(terrain.elevation[index]) && layer_known(terrain.slope[index]) &&
      layer_known(terrain.roughness[index]) && layer_known(terrain.traversability[index]))
    {
      // Known unsafe terrain also ends the hold so new obstacles are published immediately.
      ++feature_ready_cells;
      if (in_start_radius) {++start_feature_ready_cells;}
    }
    maximum_observation_count = std::max(maximum_observation_count, count);
  }
  const bool current_scan_contributed = last_in_map_cell_count_ > 0U;
  if (last_published_start_feature_ready_ && current_scan_contributed &&
    start_feature_ready_cells == 0U)
  {
    confidence_rebuild_active_ = true;
    ++suppressed_confidence_rebuild_maps_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Suppressing confidence-rebuild terrain map: observed_cells=%zu, "
      "observation_ready_cells=%zu, start_observation_ready_cells=%zu, "
      "feature_ready_cells=%zu, "
      "start_feature_ready_cells=%zu, max_observation_count=%u, "
      "min_observed_frames=%d, accepted_points=%zu, in_map_cells=%zu, "
      "source_delta=%.3f s, receive_gap=%.3f s, suppressed_maps=%zu; "
      "the last published map keeps its original source timestamp",
      observed_cells, observation_ready_cells, start_observation_ready_cells,
      feature_ready_cells, start_feature_ready_cells,
      static_cast<unsigned int>(maximum_observation_count), min_observed_frames_,
      last_accepted_point_count_, last_in_map_cell_count_,
      have_cloud_interval_ ? last_cloud_source_delta_ : 0.0,
      have_cloud_interval_ ? last_cloud_receive_gap_ : 0.0,
      suppressed_confidence_rebuild_maps_);
    return;
  }
  if (confidence_rebuild_active_) {
    RCLCPP_INFO(
      get_logger(),
      "Terrain confidence rebuild ended: observed_cells=%zu, observation_ready_cells=%zu, "
      "start_observation_ready_cells=%zu, feature_ready_cells=%zu, "
      "start_feature_ready_cells=%zu, in_map_cells=%zu, suppressed_maps=%zu",
      observed_cells, observation_ready_cells, start_observation_ready_cells,
      feature_ready_cells, start_feature_ready_cells, last_in_map_cell_count_,
      suppressed_confidence_rebuild_maps_);
    confidence_rebuild_active_ = false;
    suppressed_confidence_rebuild_maps_ = 0U;
  }
  last_published_start_feature_ready_ = start_feature_ready_cells > 0U;
  terrain_pub_->publish(terrain);
  cost_pub_->publish(makeCostmap(terrain));
}

nav_msgs::msg::OccupancyGrid TerrainMapperNode::makeCostmap(
  const utree_dog_msgs::msg::TerrainGrid & terrain) const
{
  nav_msgs::msg::OccupancyGrid result;
  result.header = terrain.header;
  result.info.resolution = terrain.resolution;
  result.info.width = terrain.width;
  result.info.height = terrain.height;
  result.info.origin.position.x = terrain.origin_x;
  result.info.origin.position.y = terrain.origin_y;
  result.info.origin.orientation.w = 1.0;
  if (flat_obstacle_mode_) {
    const auto inflated = flat_map_builder_->inflatedObstacleMask();
    result.data.resize(inflated.size(), 0);
    std::transform(
      inflated.begin(), inflated.end(), result.data.begin(),
      [](std::uint8_t occupied) {
        return static_cast<std::int8_t>(occupied != 0U ? 100 : 0);
      });
    return result;
  }
  result.data.resize(terrain.traversability.size(), -1);
  for (std::size_t i = 0; i < terrain.traversability.size(); ++i) {
    if (terrain.traversability[i] != terrain.unknown_value) {
      result.data[i] = static_cast<std::int8_t>(std::lround(
        100.0 * (1.0 - std::clamp(terrain.traversability[i], 0.0F, 1.0F))));
    }
  }
  return result;
}

nav_msgs::msg::GridCells TerrainMapperNode::makeFlatCells(
  const std::vector<std::uint8_t> & mask,
  const builtin_interfaces::msg::Time & stamp, double z_offset) const
{
  nav_msgs::msg::GridCells result;
  result.header.stamp = stamp;
  result.header.frame_id = map_frame_;
  const auto & config = flat_map_builder_->mapConfig();
  result.cell_width = config.resolution;
  result.cell_height = config.resolution;
  result.cells.reserve(static_cast<std::size_t>(std::count_if(
      mask.begin(), mask.end(), [](std::uint8_t value) {return value != 0U;})));
  for (std::size_t index = 0; index < mask.size(); ++index) {
    if (mask[index] == 0U) {
      continue;
    }
    geometry_msgs::msg::Point point;
    point.x = config.origin_x +
      (static_cast<double>(index % flat_map_builder_->width()) + 0.5) * config.resolution;
    point.y = config.origin_y +
      (static_cast<double>(index / flat_map_builder_->width()) + 0.5) * config.resolution;
    point.z = flat_ground_z_ + z_offset;
    result.cells.push_back(point);
  }
  return result;
}

}  // namespace utree_dog_navigation
