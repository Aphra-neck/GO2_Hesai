#include "utree_dog_navigation/terrain_mapper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace utree_dog_navigation
{
namespace
{
constexpr double kQuaternionNormEpsilon = 1.0e-12;
constexpr std::size_t kFlatOdomHistoryLimit = 32U;
constexpr std::size_t kPendingFlatCloudLimit = 8U;
constexpr std::chrono::milliseconds kPendingFlatCloudMaxWait{250};
constexpr double kFlatPoseJumpTranslation = 1.0;
constexpr double kFlatPoseJumpAngle = 0.7853981633974483;

struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};

Quaternion multiply(const Quaternion & left, const Quaternion & right) noexcept
{
  return {
    left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
    left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
    left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
    left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z};
}

TerrainPoint rotate(const Quaternion & orientation, const TerrainPoint & point) noexcept
{
  const TerrainPoint twice_cross{
    2.0 * (orientation.y * point.z - orientation.z * point.y),
    2.0 * (orientation.z * point.x - orientation.x * point.z),
    2.0 * (orientation.x * point.y - orientation.y * point.x)};
  return {
    point.x + orientation.w * twice_cross.x +
    orientation.y * twice_cross.z - orientation.z * twice_cross.y,
    point.y + orientation.w * twice_cross.y +
    orientation.z * twice_cross.x - orientation.x * twice_cross.z,
    point.z + orientation.w * twice_cross.z +
    orientation.x * twice_cross.y - orientation.y * twice_cross.x};
}

bool validRosTimestamp(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U &&
         (stamp.sec != 0 || stamp.nanosec != 0U);
}

bool sameTimestamp(
  const builtin_interfaces::msg::Time & left,
  const builtin_interfaces::msg::Time & right) noexcept
{
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

int compareTimestamp(
  const builtin_interfaces::msg::Time & left,
  const builtin_interfaces::msg::Time & right) noexcept
{
  if (left.sec != right.sec) {
    return left.sec < right.sec ? -1 : 1;
  }
  if (left.nanosec == right.nanosec) {
    return 0;
  }
  return left.nanosec < right.nanosec ? -1 : 1;
}

double timestampSeconds(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
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
  FlatObstacleVoxelConfig flat_config;
  flat_config.min_height = declare_parameter("flat_obstacle.min_height", 0.08);
  flat_config.max_height = declare_parameter("flat_obstacle.max_height", 0.80);
  flat_config.voxel_height = declare_parameter("flat_obstacle.voxel_height", 0.10);
  flat_config.clearance = declare_parameter("flat_obstacle.obstacle_clearance", 0.10);
  const int strong_hit_points =
    declare_parameter("flat_obstacle.strong_hit_points", 3);
  const int hit_confirmation_frames =
    declare_parameter("flat_obstacle.hit_confirmation_frames", 2);
  flat_config.hit_confirmation_window =
    declare_parameter("flat_obstacle.hit_confirmation_window", 0.35);
  const int clear_confirmation_frames =
    declare_parameter("flat_obstacle.clear_confirmation_frames", 2);
  flat_config.clear_confirmation_window =
    declare_parameter("flat_obstacle.clear_confirmation_window", 0.35);
  if (strong_hit_points < 1 || hit_confirmation_frames < 1 ||
    clear_confirmation_frames < 1)
  {
    throw std::invalid_argument(
            "flat obstacle voxel evidence counts must be greater than zero");
  }
  flat_config.strong_hit_points = static_cast<std::size_t>(strong_hit_points);
  flat_config.hit_confirmation_frames =
    static_cast<std::size_t>(hit_confirmation_frames);
  flat_config.clear_confirmation_frames =
    static_cast<std::size_t>(clear_confirmation_frames);
  flat_nominal_body_height_ =
    declare_parameter("flat_obstacle.nominal_body_height", 0.34);
  flat_max_odom_age_ = declare_parameter("flat_obstacle.max_odom_age", 0.5);
  body_yaw_offset_ = declare_parameter("body_yaw_offset", -1.5707963267948966);
  lidar_offset_x_ = declare_parameter("flat_obstacle.lidar_offset.x", 0.171);
  lidar_offset_y_ = declare_parameter("flat_obstacle.lidar_offset.y", 0.0);
  lidar_offset_z_ = declare_parameter("flat_obstacle.lidar_offset.z", 0.0908);
  visualization_voxel_size_ =
    declare_parameter("flat_obstacle.visualization.voxel_size", 0.30);
  const int visualization_max_points =
    declare_parameter("flat_obstacle.visualization.max_points", 5000);
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
  if (!std::isfinite(body_yaw_offset_) || std::abs(body_yaw_offset_) > 3.141592653589793 ||
    !std::isfinite(lidar_offset_x_) || !std::isfinite(lidar_offset_y_) ||
    !std::isfinite(lidar_offset_z_) ||
    !std::isfinite(visualization_voxel_size_) || visualization_voxel_size_ <= 0.0 ||
    visualization_max_points < 1)
  {
    throw std::invalid_argument("flat obstacle sensor or visualization configuration is invalid");
  }
  visualization_max_points_ = static_cast<std::size_t>(visualization_max_points);
  if (flat_obstacle_mode_) {
    flat_voxel_map_ = std::make_unique<FlatObstacleVoxelMap>(config, flat_config);
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
    flat_voxel_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "flat_obstacle_confirmed_voxels",
      rclcpp::QoS(1).best_effort().durability_volatile());
    flat_map_3d_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "flat_obstacle_map_3d",
      rclcpp::QoS(8).reliable().durability_volatile());
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
    flat_obstacle_mode_ ? flat_voxel_map_->width() : map_builder_->width(),
    flat_obstacle_mode_ ? flat_voxel_map_->height() : map_builder_->height());
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
  if (flat_obstacle_mode_) {
    FlatBodyPose pose;
    pose.stamp = msg->header.stamp;
    pose.x = position.x;
    pose.y = position.y;
    pose.z = position.z;
    pose.yaw = robot_yaw_;
    pose.qx = x;
    pose.qy = y;
    pose.qz = z;
    pose.qw = w;
    double pose_translation = 0.0;
    double pose_angle = 0.0;
    if (!flatPoseContinuous(pose, pose_translation, pose_angle)) {
      RCLCPP_WARN(
        get_logger(),
        "Flat obstacle body pose jumped by %.3f m / %.3f rad; invalidating the old map epoch",
        pose_translation, pose_angle);
      invalidateFlatEpoch(pose);
    }
    cacheFlatPose(pose);
    if (!flat_ground_locked_) {
      flat_ground_z_ = pose.z - flat_nominal_body_height_;
      flat_ground_locked_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Flat obstacle ground locked at z=%.3f m from standing body z=%.3f m; "
        "keep the robot standing while this mode is active",
        flat_ground_z_, pose.z);
    }
    processPendingFlatClouds(pose);
  }
}

void TerrainMapperNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
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
  const auto received_at = std::chrono::steady_clock::now();
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
    prunePendingFlatClouds(received_at);
    const FlatBodyPose * pose = findFlatPose(msg->header.stamp);
    if (pose == nullptr) {
      queueFlatCloud(msg, received_at);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Queued flat obstacle cloud until body odometry with the exact source timestamp arrives");
      return;
    }
    processCloud(msg, pose, received_at);
    return;
  }

  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for odometry");
    return;
  }
  processCloud(msg, nullptr, received_at);
}

void TerrainMapperNode::processCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
  const FlatBodyPose * flat_pose,
  std::chrono::steady_clock::time_point received_at)
{
  FlatBodyPose matched_flat_pose;
  if (flat_obstacle_mode_) {
    if (flat_pose == nullptr || !flat_ground_locked_) {
      return;
    }
    matched_flat_pose = *flat_pose;
    flat_pose = &matched_flat_pose;
    const rclcpp::Time current_time = now();
    const rclcpp::Time cloud_time(msg->header.stamp, current_time.get_clock_type());
    const double header_age = (current_time - cloud_time).seconds();
    if (!std::isfinite(header_age) || header_age > cloud_stale_warning_age_ ||
      header_age < -0.2)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Dropped queued flat obstacle cloud before processing: header_age=%.3f s, "
        "threshold=%.3f s",
        header_age, cloud_stale_warning_age_);
      return;
    }
  }

  const double stamp_seconds = timestampSeconds(msg->header.stamp);
  int source_stamp_order = 1;
  if (last_cloud_received_ != std::chrono::steady_clock::time_point{} &&
    validRosTimestamp(last_cloud_stamp_))
  {
    const double previous_stamp_seconds = timestampSeconds(last_cloud_stamp_);
    last_cloud_source_delta_ = stamp_seconds - previous_stamp_seconds;
    last_cloud_receive_gap_ =
      std::chrono::duration<double>(received_at - last_cloud_received_).count();
    have_cloud_interval_ = true;
    source_stamp_order = compareTimestamp(msg->header.stamp, last_cloud_stamp_);
    if (!std::isfinite(last_cloud_source_delta_) || source_stamp_order == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected nonmonotonic terrain source cloud timestamp: source_delta=%.3f s, "
        "receive_gap=%.3f s",
        last_cloud_source_delta_, last_cloud_receive_gap_);
      return;
    }
    if (source_stamp_order < 0 && !flat_obstacle_mode_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected nonmonotonic terrain source cloud timestamp: source_delta=%.3f s, "
        "receive_gap=%.3f s",
        last_cloud_source_delta_, last_cloud_receive_gap_);
      return;
    }
    if (source_stamp_order < 0) {
      RCLCPP_WARN(
        get_logger(),
        "Flat obstacle source clock moved backward by %.3f s; starting a new voxel epoch",
        -last_cloud_source_delta_);
      invalidateFlatEpoch(*flat_pose);
    }
    const bool exceeds_integration_window =
      source_stamp_order > 0 && last_cloud_source_delta_ > integration_window_;
    if (exceeds_integration_window) {
      RCLCPP_WARN(
        get_logger(),
        "Terrain source cloud timestamp discontinuity: source_delta=%.3f s, "
        "receive_gap=%.3f s, integration_window=%.3f s",
        last_cloud_source_delta_, last_cloud_receive_gap_, integration_window_);
    }
  }

  const double pose_x = flat_pose == nullptr ? robot_x_ : flat_pose->x;
  const double pose_y = flat_pose == nullptr ? robot_y_ : flat_pose->y;
  const double pose_z = flat_pose == nullptr ? robot_z_ : flat_pose->z;
  const double pose_yaw = flat_pose == nullptr ? robot_yaw_ : flat_pose->yaw;
  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
  std::vector<TerrainPoint> accepted_points;
  accepted_points.reserve(msg->width * msg->height / 2);
  const double cos_yaw = std::cos(pose_yaw);
  const double sin_yaw = std::sin(pose_yaw);
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
    const double dx = *x - pose_x;
    const double dy = *y - pose_y;
    const double dz = *z - pose_z;
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
  if (flat_obstacle_mode_ && accepted_points.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected empty flat obstacle point cloud after range, height, and self filtering");
    return;
  }
  bool flat_timing_continuous = false;
  if (flat_obstacle_mode_ && have_cloud_interval_) {
    flat_timing_continuous =
      source_stamp_order > 0 && last_cloud_source_delta_ <= cloud_stale_warning_age_ &&
      last_cloud_source_delta_ <= integration_window_ && last_cloud_receive_gap_ >= 0.0 &&
      last_cloud_receive_gap_ <= cloud_stale_warning_age_;
  }
  if (flat_obstacle_mode_) {
    double pose_translation = 0.0;
    double pose_angle = 0.0;
    const bool pose_continuous =
      flatPoseContinuous(*flat_pose, pose_translation, pose_angle);
    const auto update = flat_voxel_map_->update(
      accepted_points, flatSensorOrigin(*flat_pose), stamp_seconds, flat_ground_z_,
      flat_timing_continuous && pose_continuous);
    if (!update.accepted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected flat obstacle voxel update with invalid or nonmonotonic input");
      return;
    }
    last_processed_flat_pose_ = *flat_pose;
    have_processed_flat_pose_ = true;
    last_in_map_cell_count_ = update.endpoint_voxels;
  } else {
    last_in_map_cell_count_ = map_builder_->integrateFrame(accepted_points, stamp_seconds);
  }
  last_accepted_point_count_ = accepted_points.size();
  last_cloud_stamp_ = msg->header.stamp;
  last_cloud_received_ = received_at;
}

void TerrainMapperNode::cacheFlatPose(const FlatBodyPose & pose)
{
  const auto existing = std::find_if(
    flat_odom_history_.begin(), flat_odom_history_.end(),
    [&pose](const FlatBodyPose & candidate) {
      return sameTimestamp(candidate.stamp, pose.stamp);
    });
  if (existing != flat_odom_history_.end()) {
    *existing = pose;
    return;
  }
  flat_odom_history_.push_back(pose);
  while (flat_odom_history_.size() > kFlatOdomHistoryLimit) {
    flat_odom_history_.pop_front();
  }
}

const TerrainMapperNode::FlatBodyPose * TerrainMapperNode::findFlatPose(
  const builtin_interfaces::msg::Time & stamp) const
{
  const auto found = std::find_if(
    flat_odom_history_.rbegin(), flat_odom_history_.rend(),
    [&stamp](const FlatBodyPose & pose) {return sameTimestamp(pose.stamp, stamp);});
  return found == flat_odom_history_.rend() ? nullptr : &*found;
}

void TerrainMapperNode::queueFlatCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
  std::chrono::steady_clock::time_point received_at)
{
  const auto duplicate = std::find_if(
    pending_flat_clouds_.begin(), pending_flat_clouds_.end(),
    [&msg](const PendingFlatCloud & pending) {
      return sameTimestamp(pending.message->header.stamp, msg->header.stamp);
    });
  if (duplicate != pending_flat_clouds_.end()) {
    return;
  }
  if (pending_flat_clouds_.size() >= kPendingFlatCloudLimit) {
    pending_flat_clouds_.pop_front();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Dropped oldest unmatched flat obstacle cloud because the bounded queue is full");
  }
  pending_flat_clouds_.push_back({msg, received_at});
}

void TerrainMapperNode::processPendingFlatClouds(const FlatBodyPose & pose)
{
  const auto current_time = std::chrono::steady_clock::now();
  prunePendingFlatClouds(current_time);
  for (auto pending = pending_flat_clouds_.begin(); pending != pending_flat_clouds_.end();) {
    if (!sameTimestamp(pending->message->header.stamp, pose.stamp)) {
      ++pending;
      continue;
    }
    PendingFlatCloud matched = std::move(*pending);
    pending = pending_flat_clouds_.erase(pending);
    processCloud(matched.message, &pose, matched.received_at);
    return;
  }
}

void TerrainMapperNode::prunePendingFlatClouds(
  std::chrono::steady_clock::time_point current_time)
{
  std::size_t dropped = 0U;
  while (!pending_flat_clouds_.empty() &&
    current_time - pending_flat_clouds_.front().received_at > kPendingFlatCloudMaxWait)
  {
    pending_flat_clouds_.pop_front();
    ++dropped;
  }
  if (dropped != 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Dropped %zu flat obstacle cloud(s) without exact-timestamp body odometry",
      dropped);
  }
}

bool TerrainMapperNode::flatPoseContinuous(
  const FlatBodyPose & pose, double & translation, double & angle) const
{
  translation = 0.0;
  angle = 0.0;
  if (!have_processed_flat_pose_) {
    return true;
  }
  translation = std::hypot(
    std::hypot(
      pose.x - last_processed_flat_pose_.x,
      pose.y - last_processed_flat_pose_.y),
    pose.z - last_processed_flat_pose_.z);
  const double quaternion_dot = std::abs(
    pose.qx * last_processed_flat_pose_.qx +
    pose.qy * last_processed_flat_pose_.qy +
    pose.qz * last_processed_flat_pose_.qz +
    pose.qw * last_processed_flat_pose_.qw);
  angle = 2.0 * std::acos(std::clamp(quaternion_dot, 0.0, 1.0));
  return translation <= kFlatPoseJumpTranslation && angle <= kFlatPoseJumpAngle;
}

void TerrainMapperNode::invalidateFlatEpoch(const FlatBodyPose & pose)
{
  flat_voxel_map_->resetEpoch();
  flat_odom_history_.clear();
  pending_flat_clouds_.clear();
  have_processed_flat_pose_ = false;
  have_cloud_interval_ = false;
  last_cloud_received_ = std::chrono::steady_clock::time_point{};
  last_cloud_stamp_ = builtin_interfaces::msg::Time{};
  last_accepted_point_count_ = 0U;
  last_in_map_cell_count_ = 0U;
  flat_ground_z_ = pose.z - flat_nominal_body_height_;
  flat_ground_locked_ = true;

  utree_dog_msgs::msg::TerrainGrid invalid_map;
  invalid_map.header.stamp = pose.stamp;
  invalid_map.header.frame_id = map_frame_;
  terrain_pub_->publish(invalid_map);
  const std::vector<std::uint8_t> empty_mask;
  flat_raw_pub_->publish(makeFlatCells(empty_mask, pose.stamp, 0.02));
  flat_inflated_pub_->publish(makeFlatCells(empty_mask, pose.stamp, 0.01));
  cost_pub_->publish(makeUnknownFlatCostmap(pose.stamp));
  if (flat_voxel_pub_->get_subscription_count() > 0U) {
    flat_voxel_pub_->publish(makeVoxelCloud({}, pose.stamp, true));
  }
  RCLCPP_WARN(
    get_logger(),
    "Relocked flat obstacle ground at world z=%.3f m; waiting for a new exact-stamp cloud/odom pair",
    flat_ground_z_);
}

void TerrainMapperNode::publishMap()
{
  if (flat_obstacle_mode_) {
    prunePendingFlatClouds(std::chrono::steady_clock::now());
  }
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
    const std::vector<std::uint8_t> empty_mask;
    flat_raw_pub_->publish(makeFlatCells(empty_mask, last_cloud_stamp_, 0.02));
    flat_inflated_pub_->publish(makeFlatCells(empty_mask, last_cloud_stamp_, 0.01));
    cost_pub_->publish(makeUnknownFlatCostmap(last_cloud_stamp_));
    if (flat_voxel_pub_->get_subscription_count() > 0U) {
      flat_voxel_pub_->publish(makeVoxelCloud({}, last_cloud_stamp_, true));
    }
    return;
  }
  if (flat_obstacle_mode_) {
    const bool include_voxel_centers =
      flat_voxel_pub_->get_subscription_count() > 0U ||
      flat_map_3d_pub_->get_subscription_count() > 0U;
    const auto snapshot = flat_voxel_map_->snapshot(
      last_cloud_stamp_, map_frame_, flat_ground_z_, include_voxel_centers);
    terrain_pub_->publish(snapshot.terrain);
    cost_pub_->publish(makeCostmap(snapshot.terrain, &snapshot.inflated_obstacles));
    flat_inflated_pub_->publish(makeFlatCells(
        snapshot.inflated_obstacles, last_cloud_stamp_, 0.01));
    flat_raw_pub_->publish(makeFlatCells(
        snapshot.raw_obstacles, last_cloud_stamp_, 0.02));
    if (flat_voxel_pub_->get_subscription_count() > 0U) {
      flat_voxel_pub_->publish(makeVoxelCloud(
          snapshot.confirmed_voxel_centers, last_cloud_stamp_, true));
    }
    if (flat_map_3d_pub_->get_subscription_count() > 0U) {
      flat_map_3d_pub_->publish(makeVoxelCloud(
          snapshot.confirmed_voxel_centers, last_cloud_stamp_, false));
    }
    return;
  }
  const auto terrain = map_builder_->build(last_cloud_stamp_, map_frame_);
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
  const utree_dog_msgs::msg::TerrainGrid & terrain,
  const std::vector<std::uint8_t> * inflated_obstacles) const
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
    if (inflated_obstacles == nullptr) {
      throw std::invalid_argument("flat obstacle costmap requires an inflated mask");
    }
    result.data.resize(inflated_obstacles->size(), 0);
    std::transform(
      inflated_obstacles->begin(), inflated_obstacles->end(), result.data.begin(),
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
  const auto & config = flat_voxel_map_->mapConfig();
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
      (static_cast<double>(index % flat_voxel_map_->width()) + 0.5) * config.resolution;
    point.y = config.origin_y +
      (static_cast<double>(index / flat_voxel_map_->width()) + 0.5) * config.resolution;
    point.z = flat_ground_z_ + z_offset;
    result.cells.push_back(point);
  }
  return result;
}

nav_msgs::msg::OccupancyGrid TerrainMapperNode::makeUnknownFlatCostmap(
  const builtin_interfaces::msg::Time & stamp) const
{
  nav_msgs::msg::OccupancyGrid result;
  result.header.stamp = stamp;
  result.header.frame_id = map_frame_;
  const auto & config = flat_voxel_map_->mapConfig();
  result.info.resolution = config.resolution;
  result.info.width = static_cast<std::uint32_t>(flat_voxel_map_->width());
  result.info.height = static_cast<std::uint32_t>(flat_voxel_map_->height());
  result.info.origin.position.x = config.origin_x;
  result.info.origin.position.y = config.origin_y;
  result.info.origin.orientation.w = 1.0;
  result.data.assign(
    flat_voxel_map_->width() * flat_voxel_map_->height(),
    static_cast<std::int8_t>(-1));
  return result;
}

sensor_msgs::msg::PointCloud2 TerrainMapperNode::makeVoxelCloud(
  const std::vector<TerrainPoint> & voxel_centers,
  const builtin_interfaces::msg::Time & stamp, bool downsample) const
{
  std::vector<const TerrainPoint *> selected;
  if (downsample) {
    double leaf_size = visualization_voxel_size_;
    do {
      selected.clear();
      std::unordered_set<std::uint64_t> occupied_leaves;
      occupied_leaves.reserve(voxel_centers.size());
      for (const auto & point : voxel_centers) {
        const auto x = static_cast<std::uint64_t>(std::max(0.0, std::floor(
              (point.x - flat_voxel_map_->mapConfig().origin_x) / leaf_size)));
        const auto y = static_cast<std::uint64_t>(std::max(0.0, std::floor(
              (point.y - flat_voxel_map_->mapConfig().origin_y) / leaf_size)));
        const auto z = static_cast<std::uint64_t>(std::max(0.0, std::floor(
              (point.z - flat_ground_z_) / leaf_size)));
        const std::uint64_t key = (x << 42U) | (y << 21U) | z;
        if (occupied_leaves.insert(key).second) {
          selected.push_back(&point);
        }
      }
      if (selected.size() <= visualization_max_points_) {
        break;
      }
      leaf_size *= 2.0;
    } while (std::isfinite(leaf_size));
  } else {
    selected.reserve(voxel_centers.size());
    for (const auto & point : voxel_centers) {
      selected.push_back(&point);
    }
  }

  sensor_msgs::msg::PointCloud2 result;
  result.header.stamp = stamp;
  result.header.frame_id = map_frame_;
  sensor_msgs::PointCloud2Modifier modifier(result);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(selected.size());
  sensor_msgs::PointCloud2Iterator<float> x(result, "x");
  sensor_msgs::PointCloud2Iterator<float> y(result, "y");
  sensor_msgs::PointCloud2Iterator<float> z(result, "z");
  for (const TerrainPoint * point : selected) {
    *x = static_cast<float>(point->x);
    *y = static_cast<float>(point->y);
    *z = static_cast<float>(point->z);
    ++x;
    ++y;
    ++z;
  }
  result.is_dense = true;
  return result;
}

TerrainPoint TerrainMapperNode::flatSensorOrigin(const FlatBodyPose & pose) const
{
  const Quaternion world_from_body{pose.qx, pose.qy, pose.qz, pose.qw};
  const double half_inverse_offset = -0.5 * body_yaw_offset_;
  const Quaternion body_from_imu{
    0.0, 0.0, std::sin(half_inverse_offset), std::cos(half_inverse_offset)};
  const Quaternion world_from_imu = multiply(world_from_body, body_from_imu);
  const TerrainPoint offset = rotate(
    world_from_imu, {lidar_offset_x_, lidar_offset_y_, lidar_offset_z_});
  return {pose.x + offset.x, pose.y + offset.y, pose.z + offset.z};
}

}  // namespace utree_dog_navigation
