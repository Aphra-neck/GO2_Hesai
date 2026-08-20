#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "utree_dog_msgs/msg/terrain_grid.hpp"
#include "utree_dog_navigation/terrain_mapper_node.hpp"

namespace utree_dog_navigation
{
namespace
{

using namespace std::chrono_literals;

class TerrainMapperNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    const std::string suffix = std::to_string(++instance_count_);
    const std::string namespace_name = "/terrain_mapper_stamp_test_" + suffix;
    cloud_topic_ = namespace_name + "/cloud";
    odom_topic_ = namespace_name + "/odom";
    terrain_topic_ = namespace_name + "/terrain_map";

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + namespace_name});
    options.parameter_overrides({
      rclcpp::Parameter("planning_mode", "terrain"),
      rclcpp::Parameter("enable_legacy_terrain", true),
      rclcpp::Parameter("cloud_topic", cloud_topic_),
      rclcpp::Parameter("odom_topic", odom_topic_),
      rclcpp::Parameter("publish_rate", 50.0),
    });
    mapper_node_ = std::make_shared<TerrainMapperNode>(options);
    harness_node_ = std::make_shared<rclcpp::Node>("terrain_mapper_stamp_harness_" + suffix);

    cloud_pub_ = harness_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS());
    odom_pub_ = harness_node_->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS());
    terrain_sub_ = harness_node_->create_subscription<utree_dog_msgs::msg::TerrainGrid>(
      terrain_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg) {
        terrain_maps_.push_back(*msg);
      });

    executor_.add_node(mapper_node_);
    executor_.add_node(harness_node_);
    ASSERT_TRUE(spinUntil(
        [this]() {
          return cloud_pub_->get_subscription_count() == 1U &&
                 odom_pub_->get_subscription_count() == 1U &&
                 terrain_sub_->get_publisher_count() == 1U;
        },
        2s));
  }

  void TearDown() override
  {
    executor_.remove_node(harness_node_);
    executor_.remove_node(mapper_node_);
    terrain_sub_.reset();
    odom_pub_.reset();
    cloud_pub_.reset();
    harness_node_.reset();
    mapper_node_.reset();
    terrain_maps_.clear();
  }

  bool spinUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
    return predicate();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const builtin_interfaces::msg::Time & stamp) const
  {
    return makeCloud(stamp, {{{2.0F, 0.0F, 0.0F}}});
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<std::array<float, 3>> & points) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "world";
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = point[0];
      *y = point[1];
      *z = point[2];
      ++x;
      ++y;
      ++z;
    }
    return cloud;
  }

  sensor_msgs::msg::PointCloud2 makeFlatSceneCloud(
    const builtin_interfaces::msg::Time & stamp,
    double body_x = 0.0, double body_y = 0.0, double ground_z = 0.0,
    const std::vector<std::array<float, 3>> & obstacles = {}) const
  {
    std::vector<std::array<float, 3>> points;
    points.reserve(81U + obstacles.size());
    for (int y = -4; y <= 4; ++y) {
      for (int x = -4; x <= 4; ++x) {
        points.push_back({
          static_cast<float>(body_x + 0.30 * static_cast<double>(x)),
          static_cast<float>(body_y + 0.30 * static_cast<double>(y)),
          static_cast<float>(ground_z)});
      }
    }
    points.insert(points.end(), obstacles.begin(), obstacles.end());
    return makeCloud(stamp, points);
  }

  std::size_t cellIndex(
    const utree_dog_msgs::msg::TerrainGrid & terrain, double world_x, double world_y) const
  {
    const auto grid_x = static_cast<std::size_t>(
      std::floor((world_x - terrain.origin_x) / terrain.resolution));
    const auto grid_y = static_cast<std::size_t>(
      std::floor((world_y - terrain.origin_y) / terrain.resolution));
    return grid_y * terrain.width + grid_x;
  }

  std::vector<std::array<float, 3>> pointCloudPoints(
    const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    std::vector<std::array<float, 3>> points;
    points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      points.push_back({*x, *y, *z});
    }
    return points;
  }

  void expectCloudStampRejected(const builtin_interfaces::msg::Time & invalid_stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = harness_node_->now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.orientation.w = 1.0;
    odom_pub_->publish(odom);
    spinFor(50ms);

    const builtin_interfaces::msg::Time valid_stamp = harness_node_->now();
    cloud_pub_->publish(makeCloud(valid_stamp));
    ASSERT_TRUE(spinUntil(
        [this, &valid_stamp]() {
          return !terrain_maps_.empty() &&
                 terrain_maps_.back().header.stamp.sec == valid_stamp.sec &&
                 terrain_maps_.back().header.stamp.nanosec == valid_stamp.nanosec;
        },
        1s));

    terrain_maps_.clear();
    cloud_pub_->publish(makeCloud(invalid_stamp));
    EXPECT_NO_THROW(spinFor(200ms));
    ASSERT_FALSE(terrain_maps_.empty());
    EXPECT_EQ(terrain_maps_.back().header.stamp.sec, valid_stamp.sec);
    EXPECT_EQ(terrain_maps_.back().header.stamp.nanosec, valid_stamp.nanosec);
  }

  inline static int instance_count_{0};
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string terrain_topic_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<TerrainMapperNode> mapper_node_;
  rclcpp::Node::SharedPtr harness_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<utree_dog_msgs::msg::TerrainGrid>::SharedPtr terrain_sub_;
  std::vector<utree_dog_msgs::msg::TerrainGrid> terrain_maps_;
};

TEST_F(TerrainMapperNodeTest, DoesNotPublishBeforeReceivingAnAcceptedCloud)
{
  spinFor(100ms);
  EXPECT_TRUE(terrain_maps_.empty());
}

TEST_F(TerrainMapperNodeTest, RejectsNegativeCloudTimestampWithoutThrowing)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = -1;
  expectCloudStampRejected(stamp);
}

TEST_F(TerrainMapperNodeTest, RejectsOutOfRangeCloudNanosecondsWithoutThrowing)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1;
  stamp.nanosec = 1000000000U;
  expectCloudStampRejected(stamp);
}

TEST_F(TerrainMapperNodeTest, RejectsZeroCloudTimestampWithoutThrowing)
{
  expectCloudStampRejected(builtin_interfaces::msg::Time{});
}

TEST_F(TerrainMapperNodeTest, RejectsDuplicateAndBackwardCloudTimestamps)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = harness_node_->now();
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.orientation.w = 1.0;
  odom_pub_->publish(odom);
  spinFor(50ms);

  const builtin_interfaces::msg::Time valid_stamp = harness_node_->now();
  const std::vector<std::array<float, 3>> points{{{2.05F, 0.05F, 0.0F}}};
  cloud_pub_->publish(makeCloud(valid_stamp, points));
  ASSERT_TRUE(spinUntil(
      [this, &valid_stamp]() {
        return !terrain_maps_.empty() &&
               terrain_maps_.back().header.stamp.sec == valid_stamp.sec &&
               terrain_maps_.back().header.stamp.nanosec == valid_stamp.nanosec;
      },
      1s));
  const std::size_t observed_cell = cellIndex(terrain_maps_.back(), 2.05, 0.05);
  ASSERT_EQ(terrain_maps_.back().observation_count[observed_cell], 1U);

  terrain_maps_.clear();
  cloud_pub_->publish(makeCloud(valid_stamp, points));
  spinFor(100ms);
  ASSERT_FALSE(terrain_maps_.empty());
  EXPECT_EQ(terrain_maps_.back().header.stamp.sec, valid_stamp.sec);
  EXPECT_EQ(terrain_maps_.back().header.stamp.nanosec, valid_stamp.nanosec);
  EXPECT_EQ(terrain_maps_.back().observation_count[observed_cell], 1U);

  terrain_maps_.clear();
  const builtin_interfaces::msg::Time earlier_stamp =
    rclcpp::Time(valid_stamp) - rclcpp::Duration::from_seconds(0.1);
  cloud_pub_->publish(makeCloud(earlier_stamp, points));
  spinFor(100ms);
  ASSERT_FALSE(terrain_maps_.empty());
  EXPECT_EQ(terrain_maps_.back().header.stamp.sec, valid_stamp.sec);
  EXPECT_EQ(terrain_maps_.back().header.stamp.nanosec, valid_stamp.nanosec);
  EXPECT_EQ(terrain_maps_.back().observation_count[observed_cell], 1U);
}

TEST_F(TerrainMapperNodeTest, AppliesSelfFilterInBodyFrameAtNinetyDegreeYaw)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = harness_node_->now();
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.orientation.z = std::sqrt(0.5);
  odom.pose.pose.orientation.w = std::sqrt(0.5);
  odom_pub_->publish(odom);
  spinFor(50ms);

  const builtin_interfaces::msg::Time cloud_stamp = harness_node_->now();
  cloud_pub_->publish(makeCloud(
      cloud_stamp,
      {{{0.01F, 0.39F, 0.0F}, {-0.39F, 0.01F, 0.0F}}}));
  ASSERT_TRUE(spinUntil(
      [this, &cloud_stamp]() {
        return !terrain_maps_.empty() &&
               terrain_maps_.back().header.stamp.sec == cloud_stamp.sec &&
               terrain_maps_.back().header.stamp.nanosec == cloud_stamp.nanosec;
      },
      1s));

  const auto & terrain = terrain_maps_.back();
  const std::size_t inside_rotated_length = cellIndex(terrain, 0.01, 0.39);
  const std::size_t outside_rotated_width = cellIndex(terrain, -0.39, 0.01);
  ASSERT_LT(inside_rotated_length, terrain.observation_count.size());
  ASSERT_LT(outside_rotated_width, terrain.observation_count.size());
  EXPECT_EQ(terrain.observation_count[inside_rotated_length], 0U);
  EXPECT_EQ(terrain.observation_count[outside_rotated_width], 1U);
}

TEST_F(TerrainMapperNodeTest, DoesNotPublishFreshEmptyMapWhileConfidenceRebuilds)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = harness_node_->now();
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.orientation.w = 1.0;
  odom_pub_->publish(odom);
  spinFor(50ms);

  const rclcpp::Time base_time = harness_node_->now();
  const std::vector<std::array<float, 3>> start_patch{
    {{-0.10F, 0.30F, 0.0F}}, {{0.10F, 0.30F, 0.0F}}, {{0.30F, 0.30F, 0.0F}},
    {{-0.10F, 0.50F, 0.0F}}, {{0.10F, 0.50F, 0.0F}}, {{0.30F, 0.50F, 0.0F}},
    {{-0.10F, 0.70F, 0.0F}}, {{0.10F, 0.70F, 0.0F}}, {{0.30F, 0.70F, 0.0F}},
  };
  const std::vector<std::array<float, 3>> remote_patch{
    {{1.85F, -0.15F, 0.0F}}, {{2.05F, -0.15F, 0.0F}}, {{2.25F, -0.15F, 0.0F}},
    {{1.85F, 0.05F, 0.0F}}, {{2.05F, 0.05F, 0.0F}}, {{2.25F, 0.05F, 0.0F}},
    {{1.85F, 0.25F, 0.0F}}, {{2.05F, 0.25F, 0.0F}}, {{2.25F, 0.25F, 0.0F}},
  };
  auto combined_patch = start_patch;
  combined_patch.insert(combined_patch.end(), remote_patch.begin(), remote_patch.end());
  builtin_interfaces::msg::Time ready_stamp;
  for (int frame = 0; frame < 4; ++frame) {
    ready_stamp = base_time + rclcpp::Duration::from_seconds(frame * 0.1);
    cloud_pub_->publish(makeCloud(ready_stamp, start_patch));
    ASSERT_TRUE(spinUntil(
        [this, &ready_stamp]() {
          if (terrain_maps_.empty()) {return false;}
          const auto & map = terrain_maps_.back();
          return map.header.stamp.sec == ready_stamp.sec &&
                 map.header.stamp.nanosec == ready_stamp.nanosec;
        },
        1s));
  }

  const std::size_t observed_cell = cellIndex(terrain_maps_.back(), 0.10, 0.50);
  ASSERT_GE(terrain_maps_.back().observation_count[observed_cell], 4U);
  ASSERT_NE(
    terrain_maps_.back().traversability[observed_cell], terrain_maps_.back().unknown_value);

  // These three 0.6 s gaps expire the original frames without a single 1.5 s discontinuity.
  builtin_interfaces::msg::Time last_ready_stamp;
  for (const double seconds : {0.9, 1.5}) {
    last_ready_stamp = base_time + rclcpp::Duration::from_seconds(seconds);
    cloud_pub_->publish(makeCloud(last_ready_stamp, start_patch));
    ASSERT_TRUE(spinUntil(
        [this, &last_ready_stamp]() {
          if (terrain_maps_.empty()) {return false;}
          const auto & map = terrain_maps_.back();
          return map.header.stamp.sec == last_ready_stamp.sec &&
                 map.header.stamp.nanosec == last_ready_stamp.nanosec;
        },
        1s));
  }

  terrain_maps_.clear();
  const builtin_interfaces::msg::Time rebuilding_stamp =
    base_time + rclcpp::Duration::from_seconds(2.1);
  cloud_pub_->publish(makeCloud(rebuilding_stamp, start_patch));
  spinFor(100ms);

  EXPECT_FALSE(std::any_of(
      terrain_maps_.begin(), terrain_maps_.end(),
      [&rebuilding_stamp](const auto & map) {
        return map.header.stamp.sec == rebuilding_stamp.sec &&
               map.header.stamp.nanosec == rebuilding_stamp.nanosec;
      }));

  builtin_interfaces::msg::Time remote_ready_stamp;
  for (int frame = 0; frame < 4; ++frame) {
    remote_ready_stamp =
      base_time + rclcpp::Duration::from_seconds(2.2 + frame * 0.1);
    cloud_pub_->publish(makeCloud(remote_ready_stamp, remote_patch));
    spinFor(30ms);
  }
  EXPECT_FALSE(std::any_of(
      terrain_maps_.begin(), terrain_maps_.end(),
      [&remote_ready_stamp](const auto & map) {
        return map.header.stamp.sec == remote_ready_stamp.sec &&
               map.header.stamp.nanosec == remote_ready_stamp.nanosec;
      }));

  builtin_interfaces::msg::Time recovered_stamp;
  for (const double seconds : {2.6, 2.7}) {
    recovered_stamp = base_time + rclcpp::Duration::from_seconds(seconds);
    cloud_pub_->publish(makeCloud(recovered_stamp, start_patch));
    spinFor(30ms);
  }

  ASSERT_TRUE(spinUntil(
      [this, &recovered_stamp, observed_cell]() {
        if (terrain_maps_.empty()) {return false;}
        const auto & map = terrain_maps_.back();
        return map.header.stamp.sec == recovered_stamp.sec &&
               map.header.stamp.nanosec == recovered_stamp.nanosec &&
               map.observation_count[observed_cell] >= 4U;
      },
      1s));

  builtin_interfaces::msg::Time second_ready_stamp;
  for (const double seconds : {3.3, 3.9}) {
    second_ready_stamp = base_time + rclcpp::Duration::from_seconds(seconds);
    cloud_pub_->publish(makeCloud(second_ready_stamp, combined_patch));
    ASSERT_TRUE(spinUntil(
        [this, &second_ready_stamp]() {
          if (terrain_maps_.empty()) {return false;}
          const auto & map = terrain_maps_.back();
          return map.header.stamp.sec == second_ready_stamp.sec &&
                 map.header.stamp.nanosec == second_ready_stamp.nanosec;
        },
        1s));
  }

  const builtin_interfaces::msg::Time remote_refresh_stamp =
    base_time + rclcpp::Duration::from_seconds(4.1);
  cloud_pub_->publish(makeCloud(remote_refresh_stamp, remote_patch));
  ASSERT_TRUE(spinUntil(
      [this, &remote_refresh_stamp]() {
        if (terrain_maps_.empty()) {return false;}
        const auto & map = terrain_maps_.back();
        return map.header.stamp.sec == remote_refresh_stamp.sec &&
               map.header.stamp.nanosec == remote_refresh_stamp.nanosec;
      },
      1s));

  terrain_maps_.clear();
  const builtin_interfaces::msg::Time second_rebuilding_stamp =
    base_time + rclcpp::Duration::from_seconds(4.5);
  cloud_pub_->publish(makeCloud(second_rebuilding_stamp, combined_patch));
  spinFor(100ms);
  EXPECT_FALSE(std::any_of(
      terrain_maps_.begin(), terrain_maps_.end(),
      [&second_rebuilding_stamp](const auto & map) {
        return map.header.stamp.sec == second_rebuilding_stamp.sec &&
               map.header.stamp.nanosec == second_rebuilding_stamp.nanosec;
      }));

  const builtin_interfaces::msg::Time current_empty_stamp =
    base_time + rclcpp::Duration::from_seconds(4.6);
  cloud_pub_->publish(makeCloud(current_empty_stamp, {}));
  ASSERT_TRUE(spinUntil(
      [this, &current_empty_stamp]() {
        if (terrain_maps_.empty()) {return false;}
        const auto & map = terrain_maps_.back();
        return map.header.stamp.sec == current_empty_stamp.sec &&
               map.header.stamp.nanosec == current_empty_stamp.nanosec;
      },
      1s));
  EXPECT_EQ(terrain_maps_.back().observation_count[observed_cell], 3U);
  EXPECT_EQ(
    terrain_maps_.back().traversability[observed_cell], terrain_maps_.back().unknown_value);

  terrain_maps_.clear();
  const builtin_interfaces::msg::Time empty_stamp =
    base_time + rclcpp::Duration::from_seconds(6.2);
  cloud_pub_->publish(makeCloud(empty_stamp, {}));
  ASSERT_TRUE(spinUntil(
      [this, &empty_stamp]() {
        if (terrain_maps_.empty()) {return false;}
        const auto & map = terrain_maps_.back();
        return map.header.stamp.sec == empty_stamp.sec &&
               map.header.stamp.nanosec == empty_stamp.nanosec;
      },
      1s));
  EXPECT_TRUE(std::all_of(
      terrain_maps_.back().observation_count.begin(),
      terrain_maps_.back().observation_count.end(),
      [](std::uint16_t count) {return count == 0U;}));

  terrain_maps_.clear();
  const builtin_interfaces::msg::Time post_empty_stamp =
    base_time + rclcpp::Duration::from_seconds(6.3);
  cloud_pub_->publish(makeCloud(post_empty_stamp, start_patch));
  ASSERT_TRUE(spinUntil(
      [this, &post_empty_stamp]() {
        if (terrain_maps_.empty()) {return false;}
        const auto & map = terrain_maps_.back();
        return map.header.stamp.sec == post_empty_stamp.sec &&
               map.header.stamp.nanosec == post_empty_stamp.nanosec;
      },
      1s));
  EXPECT_EQ(terrain_maps_.back().observation_count[observed_cell], 1U);
}

TEST_F(TerrainMapperNodeTest, RejectsInvalidConfidenceWindowParameters)
{
  rclcpp::NodeOptions invalid_window_options;
  invalid_window_options.arguments(
    {"--ros-args", "-r", "__node:=terrain_mapper_invalid_window"});
  invalid_window_options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "terrain"),
    rclcpp::Parameter("enable_legacy_terrain", true),
    rclcpp::Parameter("integration_window", 0.0),
  });
  EXPECT_THROW(
    std::make_shared<TerrainMapperNode>(invalid_window_options), std::invalid_argument);

  rclcpp::NodeOptions invalid_frame_count_options;
  invalid_frame_count_options.arguments(
    {"--ros-args", "-r", "__node:=terrain_mapper_invalid_frame_count"});
  invalid_frame_count_options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "terrain"),
    rclcpp::Parameter("enable_legacy_terrain", true),
    rclcpp::Parameter("min_observed_frames", 65536),
  });
  EXPECT_THROW(
    std::make_shared<TerrainMapperNode>(invalid_frame_count_options), std::invalid_argument);

  rclcpp::NodeOptions invalid_radius_options;
  invalid_radius_options.arguments(
    {"--ros-args", "-r", "__node:=terrain_mapper_invalid_rebuild_radius"});
  invalid_radius_options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "terrain"),
    rclcpp::Parameter("enable_legacy_terrain", true),
    rclcpp::Parameter("confidence_rebuild.start_radius", 0.0),
  });
  EXPECT_THROW(
    std::make_shared<TerrainMapperNode>(invalid_radius_options), std::invalid_argument);
}

TEST_F(TerrainMapperNodeTest, DefaultFlatObstacleModeRequiresGroundConfirmation)
{
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", "__node:=terrain_mapper_unconfirmed_flat"});
  options.parameter_overrides({
    rclcpp::Parameter("flat_ground_confirmed", false),
  });
  EXPECT_THROW(std::make_shared<TerrainMapperNode>(options), std::invalid_argument);
}

TEST_F(TerrainMapperNodeTest, FlatObstacleModeRequiresTwoDistinctHitFrames)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=terrain_mapper_single_hit_frame"});
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("flat_ground_confirmed", true),
    rclcpp::Parameter("flat_obstacle.hit_confirmation_frames", 1),
  });
  EXPECT_THROW(std::make_shared<TerrainMapperNode>(options), std::invalid_argument);
}

TEST_F(TerrainMapperNodeTest, RejectsTerrainWithoutLegacyAuthorization)
{
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", "__node:=terrain_mapper_unauthorized_terrain"});
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "terrain"),
  });

  EXPECT_THROW(std::make_shared<TerrainMapperNode>(options), std::invalid_argument);
}

TEST_F(TerrainMapperNodeTest, RejectsLegacyAuthorizationOutsideTerrain)
{
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", "__node:=terrain_mapper_mismatched_legacy_authorization"});
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("enable_legacy_terrain", true),
    rclcpp::Parameter("flat_ground_confirmed", true),
  });

  EXPECT_THROW(std::make_shared<TerrainMapperNode>(options), std::invalid_argument);
}

TEST_F(TerrainMapperNodeTest, FlatObstacleModeAdvertisesFilteredObstacleLayers)
{
  const std::string namespace_name =
    "/flat_mapper_interface_" + std::to_string(instance_count_);
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args", "-r", "__ns:=" + namespace_name,
    "-r", "__node:=terrain_mapper",
  });
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("flat_ground_confirmed", true),
  });
  auto flat_mapper = std::make_shared<TerrainMapperNode>(options);
  executor_.add_node(flat_mapper);

  EXPECT_TRUE(spinUntil(
      [this, &namespace_name]() {
        return harness_node_->count_publishers(
          namespace_name + "/flat_obstacle_raw") == 1U &&
               harness_node_->count_publishers(
          namespace_name + "/flat_obstacle_inflated") == 1U &&
               harness_node_->count_publishers(
          namespace_name + "/flat_obstacle_filtered_points") == 1U &&
               harness_node_->count_publishers(
          namespace_name + "/flat_obstacle_filtered_map_3d") == 1U;
      },
      1s));

  const auto live_publishers = harness_node_->get_publishers_info_by_topic(
    namespace_name + "/flat_obstacle_filtered_points");
  ASSERT_EQ(live_publishers.size(), 1U);
  const auto live_qos = live_publishers.front().qos_profile().get_rmw_qos_profile();
  EXPECT_EQ(live_qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(live_qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

  const auto recorder_publishers = harness_node_->get_publishers_info_by_topic(
    namespace_name + "/flat_obstacle_filtered_map_3d");
  ASSERT_EQ(recorder_publishers.size(), 1U);
  const auto recorder_qos = recorder_publishers.front().qos_profile().get_rmw_qos_profile();
  EXPECT_EQ(recorder_qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(recorder_qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_DOUBLE_EQ(
    flat_mapper->get_parameter("flat_obstacle.voxel_resolution_z").as_double(), 0.10);

  executor_.remove_node(flat_mapper);
}

TEST_F(TerrainMapperNodeTest, FlatObstacleModeNeedsTwoExactStampFramesBeforePublishing)
{
  const std::string namespace_name =
    "/flat_mapper_data_" + std::to_string(instance_count_);
  const std::string cloud_topic = namespace_name + "/cloud";
  const std::string odom_topic = namespace_name + "/odom";
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args", "-r", "__ns:=" + namespace_name,
    "-r", "__node:=terrain_mapper",
  });
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("flat_ground_confirmed", true),
    rclcpp::Parameter("cloud_topic", cloud_topic),
    rclcpp::Parameter("odom_topic", odom_topic),
    rclcpp::Parameter("resolution", 0.2),
    rclcpp::Parameter("size_x", 4.0),
    rclcpp::Parameter("size_y", 4.0),
    rclcpp::Parameter("origin_x", -2.0),
    rclcpp::Parameter("origin_y", -2.0),
    rclcpp::Parameter("publish_rate", 50.0),
  });
  auto flat_mapper = std::make_shared<TerrainMapperNode>(options);
  auto flat_cloud_pub = harness_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS());
  auto flat_odom_pub = harness_node_->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS());
  std::vector<utree_dog_msgs::msg::TerrainGrid> maps;
  std::vector<nav_msgs::msg::GridCells> raw_layers;
  std::vector<nav_msgs::msg::GridCells> inflated_layers;
  std::vector<sensor_msgs::msg::PointCloud2> filtered_points;
  std::vector<sensor_msgs::msg::PointCloud2> filtered_maps;
  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  auto map_sub = harness_node_->create_subscription<utree_dog_msgs::msg::TerrainGrid>(
    namespace_name + "/terrain_map", map_qos,
    [&maps](const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg) {maps.push_back(*msg);});
  auto raw_sub = harness_node_->create_subscription<nav_msgs::msg::GridCells>(
    namespace_name + "/flat_obstacle_raw", map_qos,
    [&raw_layers](const nav_msgs::msg::GridCells::SharedPtr msg) {raw_layers.push_back(*msg);});
  auto inflated_sub = harness_node_->create_subscription<nav_msgs::msg::GridCells>(
    namespace_name + "/flat_obstacle_inflated", map_qos,
    [&inflated_layers](const nav_msgs::msg::GridCells::SharedPtr msg) {
      inflated_layers.push_back(*msg);
    });
  auto filtered_sub = harness_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    namespace_name + "/flat_obstacle_filtered_points", rclcpp::SensorDataQoS(),
    [&filtered_points](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_points.push_back(*msg);
    });
  auto filtered_map_sub = harness_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    namespace_name + "/flat_obstacle_filtered_map_3d",
    rclcpp::QoS(8).reliable().durability_volatile(),
    [&filtered_maps](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_maps.push_back(*msg);
    });
  executor_.add_node(flat_mapper);
  ASSERT_TRUE(spinUntil(
      [&flat_cloud_pub, &flat_odom_pub, &map_sub, &raw_sub, &inflated_sub,
        &filtered_sub, &filtered_map_sub]() {
        return flat_cloud_pub->get_subscription_count() == 1U &&
               flat_odom_pub->get_subscription_count() == 1U &&
               map_sub->get_publisher_count() == 1U &&
               raw_sub->get_publisher_count() == 1U &&
               inflated_sub->get_publisher_count() == 1U &&
               filtered_sub->get_publisher_count() == 1U &&
               filtered_map_sub->get_publisher_count() == 1U;
      },
      1s));

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.z = 0.34;
  odom.pose.pose.orientation.w = 1.0;
  const std::array<float, 3> confirmed_obstacle{{0.85F, 0.35F, 0.12F}};
  const std::array<float, 3> current_frame_only_obstacle{{-0.85F, 0.35F, 0.22F}};
  const std::vector<std::array<float, 3>> first_obstacles{{confirmed_obstacle}};

  const builtin_interfaces::msg::Time first_stamp = harness_node_->now();
  odom.header.stamp = first_stamp;
  flat_cloud_pub->publish(makeFlatSceneCloud(
      first_stamp, 0.0, 0.0, 0.0, first_obstacles));
  spinFor(30ms);
  EXPECT_TRUE(maps.empty());
  flat_odom_pub->publish(odom);
  ASSERT_TRUE(spinUntil(
      [&maps, &raw_layers, &filtered_points, &filtered_maps, &first_stamp]() {
        return !maps.empty() && maps.back().header.stamp.sec == first_stamp.sec &&
               maps.back().header.stamp.nanosec == first_stamp.nanosec &&
               maps.back().traversability.empty() && !raw_layers.empty() &&
               raw_layers.back().cells.empty() && !filtered_points.empty() &&
               filtered_points.back().width == 1U && !filtered_maps.empty() &&
               filtered_maps.back().width == 0U;
      },
      1s));

  maps.clear();
  raw_layers.clear();
  inflated_layers.clear();
  filtered_points.clear();
  filtered_maps.clear();
  spinFor(20ms);
  const builtin_interfaces::msg::Time second_stamp = harness_node_->now();
  odom.header.stamp = second_stamp;
  flat_odom_pub->publish(odom);
  const std::vector<std::array<float, 3>> second_obstacles{
    confirmed_obstacle, current_frame_only_obstacle};
  flat_cloud_pub->publish(makeFlatSceneCloud(
      second_stamp, 0.0, 0.0, 0.0, second_obstacles));
  ASSERT_TRUE(spinUntil(
      [&maps, &raw_layers, &inflated_layers, &filtered_points, &filtered_maps,
        &second_stamp]() {
        return !maps.empty() && !maps.back().traversability.empty() &&
               maps.back().header.stamp.sec == second_stamp.sec &&
               maps.back().header.stamp.nanosec == second_stamp.nanosec &&
               !raw_layers.empty() && raw_layers.back().cells.size() == 1U &&
               !inflated_layers.empty() && !inflated_layers.back().cells.empty() &&
               !filtered_points.empty() && filtered_points.back().width == 2U &&
               !filtered_maps.empty() && filtered_maps.back().width == 1U;
      },
      1s));

  const std::size_t obstacle_cell = cellIndex(maps.back(), 0.85, 0.35);
  EXPECT_NEAR(maps.back().elevation[obstacle_cell], 0.0F, 1.0e-4F);
  EXPECT_FLOAT_EQ(maps.back().traversability[obstacle_cell], 0.0F);
  const auto live_points = pointCloudPoints(filtered_points.back());
  const auto contains_point = [](const auto & points, const auto & expected) {
      return std::any_of(
        points.begin(), points.end(), [&expected](const auto & point) {
          return std::abs(point[0] - expected[0]) < 1.0e-4F &&
                 std::abs(point[1] - expected[1]) < 1.0e-4F &&
                 std::abs(point[2] - expected[2]) < 1.0e-4F;
        });
    };
  EXPECT_TRUE(contains_point(live_points, confirmed_obstacle));
  EXPECT_TRUE(contains_point(live_points, current_frame_only_obstacle));
  const auto confirmed_points = pointCloudPoints(filtered_maps.back());
  ASSERT_EQ(confirmed_points.size(), 1U);
  EXPECT_TRUE(contains_point(confirmed_points, confirmed_obstacle));

  executor_.remove_node(flat_mapper);
}

TEST_F(TerrainMapperNodeTest, FlatObstacleModeClearsOldEpochAfterPoseJump)
{
  const std::string namespace_name =
    "/flat_mapper_pose_jump_" + std::to_string(instance_count_);
  const std::string cloud_topic = namespace_name + "/cloud";
  const std::string odom_topic = namespace_name + "/odom";
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args", "-r", "__ns:=" + namespace_name,
    "-r", "__node:=terrain_mapper",
  });
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("flat_ground_confirmed", true),
    rclcpp::Parameter("cloud_topic", cloud_topic),
    rclcpp::Parameter("odom_topic", odom_topic),
    rclcpp::Parameter("resolution", 0.2),
    rclcpp::Parameter("size_x", 4.0),
    rclcpp::Parameter("size_y", 4.0),
    rclcpp::Parameter("origin_x", -2.0),
    rclcpp::Parameter("origin_y", -2.0),
    rclcpp::Parameter("publish_rate", 50.0),
  });
  auto flat_mapper = std::make_shared<TerrainMapperNode>(options);
  auto flat_cloud_pub = harness_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS());
  auto flat_odom_pub = harness_node_->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS());
  std::vector<utree_dog_msgs::msg::TerrainGrid> maps;
  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  auto map_sub = harness_node_->create_subscription<utree_dog_msgs::msg::TerrainGrid>(
    namespace_name + "/terrain_map", map_qos,
    [&maps](const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg) {maps.push_back(*msg);});
  executor_.add_node(flat_mapper);
  ASSERT_TRUE(spinUntil(
      [&flat_cloud_pub, &flat_odom_pub, &map_sub]() {
        return flat_cloud_pub->get_subscription_count() == 1U &&
               flat_odom_pub->get_subscription_count() == 1U &&
               map_sub->get_publisher_count() == 1U;
      },
      1s));

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.z = 0.34;
  odom.pose.pose.orientation.w = 1.0;
  const std::vector<std::array<float, 3>> first_obstacle{{{0.80F, 0.30F, 0.12F}}};
  for (int frame = 0; frame < 2; ++frame) {
    spinFor(20ms);
    const builtin_interfaces::msg::Time stamp = harness_node_->now();
    odom.header.stamp = stamp;
    flat_odom_pub->publish(odom);
    flat_cloud_pub->publish(makeFlatSceneCloud(stamp, 0.0, 0.0, 0.0, first_obstacle));
  }
  ASSERT_TRUE(spinUntil(
      [&maps]() {
        return !maps.empty() && !maps.back().traversability.empty() &&
               std::count(
          maps.back().traversability.begin(), maps.back().traversability.end(), 0.0F) == 1;
      },
      1s));

  maps.clear();
  spinFor(20ms);
  builtin_interfaces::msg::Time jump_stamp = harness_node_->now();
  odom.header.stamp = jump_stamp;
  odom.pose.pose.position.x = 1.5;
  odom.pose.pose.position.z = 1.34;
  flat_odom_pub->publish(odom);
  ASSERT_TRUE(spinUntil(
      [&maps, &jump_stamp]() {
        return !maps.empty() && maps.back().traversability.empty() &&
               maps.back().header.stamp.sec == jump_stamp.sec &&
               maps.back().header.stamp.nanosec == jump_stamp.nanosec;
      },
      1s));

  const std::vector<std::array<float, 3>> second_obstacle{{{2.30F, 0.30F, 1.12F}}};
  flat_cloud_pub->publish(makeFlatSceneCloud(
      jump_stamp, 1.5, 0.0, 1.0, second_obstacle));
  spinFor(30ms);
  maps.clear();
  spinFor(20ms);
  const builtin_interfaces::msg::Time recovered_stamp = harness_node_->now();
  odom.header.stamp = recovered_stamp;
  flat_odom_pub->publish(odom);
  flat_cloud_pub->publish(makeFlatSceneCloud(
      recovered_stamp, 1.5, 0.0, 1.0, second_obstacle));
  ASSERT_TRUE(spinUntil(
      [&maps, &recovered_stamp]() {
        return !maps.empty() && !maps.back().traversability.empty() &&
               maps.back().header.stamp.sec == recovered_stamp.sec &&
               maps.back().header.stamp.nanosec == recovered_stamp.nanosec;
      },
      1s));
  EXPECT_FLOAT_EQ(maps.back().traversability[cellIndex(maps.back(), 0.8, 0.3)], 1.0F);
  EXPECT_FLOAT_EQ(maps.back().traversability[cellIndex(maps.back(), 2.3, 0.3)], 0.0F);

  executor_.remove_node(flat_mapper);
}

TEST_F(TerrainMapperNodeTest, FlatObstacleSourceStampRollbackPublishesEmpty3DState)
{
  const std::string namespace_name =
    "/flat_mapper_stamp_rollback_" + std::to_string(instance_count_);
  const std::string cloud_topic = namespace_name + "/cloud";
  const std::string odom_topic = namespace_name + "/odom";
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args", "-r", "__ns:=" + namespace_name,
    "-r", "__node:=terrain_mapper",
  });
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("flat_ground_confirmed", true),
    rclcpp::Parameter("cloud_topic", cloud_topic),
    rclcpp::Parameter("odom_topic", odom_topic),
    rclcpp::Parameter("resolution", 0.2),
    rclcpp::Parameter("size_x", 4.0),
    rclcpp::Parameter("size_y", 4.0),
    rclcpp::Parameter("origin_x", -2.0),
    rclcpp::Parameter("origin_y", -2.0),
    rclcpp::Parameter("publish_rate", 100.0),
    rclcpp::Parameter("cloud_stale_warning_age", 2.0),
  });
  auto flat_mapper = std::make_shared<TerrainMapperNode>(options);
  auto flat_cloud_pub = harness_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS());
  auto flat_odom_pub = harness_node_->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS());
  std::vector<utree_dog_msgs::msg::TerrainGrid> maps;
  std::vector<sensor_msgs::msg::PointCloud2> filtered_points;
  std::vector<sensor_msgs::msg::PointCloud2> filtered_maps;
  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  auto map_sub = harness_node_->create_subscription<utree_dog_msgs::msg::TerrainGrid>(
    namespace_name + "/terrain_map", map_qos,
    [&maps](const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg) {maps.push_back(*msg);});
  auto filtered_sub = harness_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    namespace_name + "/flat_obstacle_filtered_points", rclcpp::SensorDataQoS(),
    [&filtered_points](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_points.push_back(*msg);
    });
  auto filtered_map_sub = harness_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    namespace_name + "/flat_obstacle_filtered_map_3d",
    rclcpp::QoS(8).reliable().durability_volatile(),
    [&filtered_maps](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_maps.push_back(*msg);
    });
  executor_.add_node(flat_mapper);
  ASSERT_TRUE(spinUntil(
      [&flat_cloud_pub, &flat_odom_pub, &map_sub, &filtered_sub, &filtered_map_sub]() {
        return flat_cloud_pub->get_subscription_count() == 1U &&
               flat_odom_pub->get_subscription_count() == 1U &&
               map_sub->get_publisher_count() == 1U &&
               filtered_sub->get_publisher_count() == 1U &&
               filtered_map_sub->get_publisher_count() == 1U;
      },
      1s));

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.z = 0.34;
  odom.pose.pose.orientation.w = 1.0;
  const std::vector<std::array<float, 3>> obstacle{{{0.80F, 0.30F, 0.12F}}};
  builtin_interfaces::msg::Time first_stamp;
  for (int frame = 0; frame < 2; ++frame) {
    spinFor(20ms);
    const builtin_interfaces::msg::Time stamp = harness_node_->now();
    if (frame == 0) {
      first_stamp = stamp;
    }
    odom.header.stamp = stamp;
    flat_odom_pub->publish(odom);
    flat_cloud_pub->publish(makeFlatSceneCloud(stamp, 0.0, 0.0, 0.0, obstacle));
  }
  ASSERT_TRUE(spinUntil(
      [&maps, &filtered_points, &filtered_maps]() {
        return !maps.empty() && !maps.back().traversability.empty() &&
               !filtered_points.empty() && filtered_points.back().width == 1U &&
               !filtered_maps.empty() && filtered_maps.back().width == 1U;
      },
      1s));

  maps.clear();
  filtered_points.clear();
  filtered_maps.clear();
  odom.header.stamp = first_stamp;
  flat_odom_pub->publish(odom);
  flat_cloud_pub->publish(makeFlatSceneCloud(
      first_stamp, 0.0, 0.0, 0.0, obstacle));
  ASSERT_TRUE(spinUntil(
      [&maps, &filtered_points, &filtered_maps, &first_stamp]() {
        return !maps.empty() && maps.back().traversability.empty() &&
               maps.back().header.stamp.sec == first_stamp.sec &&
               maps.back().header.stamp.nanosec == first_stamp.nanosec &&
               !filtered_points.empty() && filtered_points.back().width == 1U &&
               !filtered_maps.empty() && filtered_maps.back().width == 0U;
      },
      1s));

  const std::array<float, 3> recovered_obstacle{{-0.80F, 0.30F, 0.22F}};
  const std::vector<std::array<float, 3>> recovered_obstacles{{recovered_obstacle}};
  maps.clear();
  filtered_maps.clear();
  spinFor(20ms);
  const builtin_interfaces::msg::Time first_recovery_stamp = harness_node_->now();
  odom.header.stamp = first_recovery_stamp;
  flat_odom_pub->publish(odom);
  flat_cloud_pub->publish(makeFlatSceneCloud(
      first_recovery_stamp, 0.0, 0.0, 0.0, recovered_obstacles));
  ASSERT_TRUE(spinUntil(
      [&maps, &filtered_maps, &first_recovery_stamp]() {
        return !maps.empty() && !maps.back().traversability.empty() &&
               maps.back().header.stamp.sec == first_recovery_stamp.sec &&
               maps.back().header.stamp.nanosec == first_recovery_stamp.nanosec &&
               !filtered_maps.empty() && filtered_maps.back().width == 0U;
      },
      1s));

  maps.clear();
  filtered_maps.clear();
  spinFor(20ms);
  const builtin_interfaces::msg::Time second_recovery_stamp = harness_node_->now();
  odom.header.stamp = second_recovery_stamp;
  flat_odom_pub->publish(odom);
  flat_cloud_pub->publish(makeFlatSceneCloud(
      second_recovery_stamp, 0.0, 0.0, 0.0, recovered_obstacles));
  ASSERT_TRUE(spinUntil(
      [&maps, &filtered_maps, &second_recovery_stamp]() {
        return !maps.empty() && !maps.back().traversability.empty() &&
               maps.back().header.stamp.sec == second_recovery_stamp.sec &&
               maps.back().header.stamp.nanosec == second_recovery_stamp.nanosec &&
               !filtered_maps.empty() && filtered_maps.back().width == 1U;
      },
      1s));
  const auto recovered_points = pointCloudPoints(filtered_maps.back());
  ASSERT_EQ(recovered_points.size(), 1U);
  EXPECT_NEAR(recovered_points.front()[0], recovered_obstacle[0], 1.0e-4F);
  EXPECT_NEAR(recovered_points.front()[1], recovered_obstacle[1], 1.0e-4F);
  EXPECT_NEAR(recovered_points.front()[2], recovered_obstacle[2], 1.0e-4F);

  executor_.remove_node(flat_mapper);
}

TEST_F(TerrainMapperNodeTest, FlatObstacleUnmatchedCloudTimeoutPreservesValidatedEpoch)
{
  const std::string namespace_name =
    "/flat_mapper_unmatched_timeout_" + std::to_string(instance_count_);
  const std::string cloud_topic = namespace_name + "/cloud";
  const std::string odom_topic = namespace_name + "/odom";
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args", "-r", "__ns:=" + namespace_name,
    "-r", "__node:=terrain_mapper",
  });
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("flat_ground_confirmed", true),
    rclcpp::Parameter("cloud_topic", cloud_topic),
    rclcpp::Parameter("odom_topic", odom_topic),
    rclcpp::Parameter("resolution", 0.2),
    rclcpp::Parameter("size_x", 4.0),
    rclcpp::Parameter("size_y", 4.0),
    rclcpp::Parameter("origin_x", -2.0),
    rclcpp::Parameter("origin_y", -2.0),
    rclcpp::Parameter("publish_rate", 100.0),
    rclcpp::Parameter("cloud_stale_warning_age", 2.0),
  });
  auto flat_mapper = std::make_shared<TerrainMapperNode>(options);
  auto flat_cloud_pub = harness_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS());
  auto flat_odom_pub = harness_node_->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS());
  std::vector<utree_dog_msgs::msg::TerrainGrid> maps;
  std::vector<sensor_msgs::msg::PointCloud2> filtered_points;
  std::vector<sensor_msgs::msg::PointCloud2> filtered_maps;
  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  auto map_sub = harness_node_->create_subscription<utree_dog_msgs::msg::TerrainGrid>(
    namespace_name + "/terrain_map", map_qos,
    [&maps](const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg) {maps.push_back(*msg);});
  auto filtered_sub = harness_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    namespace_name + "/flat_obstacle_filtered_points", rclcpp::SensorDataQoS(),
    [&filtered_points](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_points.push_back(*msg);
    });
  auto filtered_map_sub = harness_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    namespace_name + "/flat_obstacle_filtered_map_3d",
    rclcpp::QoS(8).reliable().durability_volatile(),
    [&filtered_maps](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_maps.push_back(*msg);
    });
  executor_.add_node(flat_mapper);
  ASSERT_TRUE(spinUntil(
      [&flat_cloud_pub, &flat_odom_pub, &map_sub, &filtered_sub, &filtered_map_sub]() {
        return flat_cloud_pub->get_subscription_count() == 1U &&
               flat_odom_pub->get_subscription_count() == 1U &&
               map_sub->get_publisher_count() == 1U &&
               filtered_sub->get_publisher_count() == 1U &&
               filtered_map_sub->get_publisher_count() == 1U;
      },
      1s));

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.z = 0.34;
  odom.pose.pose.orientation.w = 1.0;
  const std::vector<std::array<float, 3>> obstacle{{{0.80F, 0.30F, 0.12F}}};
  builtin_interfaces::msg::Time ready_stamp;
  for (int frame = 0; frame < 2; ++frame) {
    spinFor(20ms);
    ready_stamp = harness_node_->now();
    odom.header.stamp = ready_stamp;
    flat_odom_pub->publish(odom);
    flat_cloud_pub->publish(makeFlatSceneCloud(
        ready_stamp, 0.0, 0.0, 0.0, obstacle));
  }
  ASSERT_TRUE(spinUntil(
      [&maps, &filtered_points, &filtered_maps]() {
        return !maps.empty() && !maps.back().traversability.empty() &&
               !filtered_points.empty() && filtered_points.back().width == 1U &&
               !filtered_maps.empty() && filtered_maps.back().width == 1U;
      },
      1s));

  maps.clear();
  filtered_points.clear();
  filtered_maps.clear();
  spinFor(20ms);
  const builtin_interfaces::msg::Time unmatched_stamp = harness_node_->now();
  flat_cloud_pub->publish(makeFlatSceneCloud(
      unmatched_stamp, 0.0, 0.0, 0.0, obstacle));
  for (int frame = 0; frame < 4; ++frame) {
    spinFor(100ms);
    ready_stamp = harness_node_->now();
    odom.header.stamp = ready_stamp;
    flat_odom_pub->publish(odom);
    flat_cloud_pub->publish(makeFlatSceneCloud(
        ready_stamp, 0.0, 0.0, 0.0, obstacle));
  }
  spinFor(50ms);
  ASSERT_FALSE(maps.empty());
  EXPECT_FALSE(maps.back().traversability.empty());
  EXPECT_EQ(maps.back().header.stamp.sec, ready_stamp.sec);
  EXPECT_EQ(maps.back().header.stamp.nanosec, ready_stamp.nanosec);
  ASSERT_FALSE(filtered_points.empty());
  EXPECT_EQ(filtered_points.back().width, 1U);
  ASSERT_FALSE(filtered_maps.empty());
  EXPECT_EQ(filtered_maps.back().width, 1U);

  executor_.remove_node(flat_mapper);
}

TEST_F(TerrainMapperNodeTest, FlatObstacleStaleSourcePublishesFullyFailClosedState)
{
  const std::string namespace_name =
    "/flat_mapper_stale_layers_" + std::to_string(instance_count_);
  const std::string cloud_topic = namespace_name + "/cloud";
  const std::string odom_topic = namespace_name + "/odom";
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args", "-r", "__ns:=" + namespace_name,
    "-r", "__node:=terrain_mapper",
  });
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("flat_ground_confirmed", true),
    rclcpp::Parameter("cloud_topic", cloud_topic),
    rclcpp::Parameter("odom_topic", odom_topic),
    rclcpp::Parameter("resolution", 0.2),
    rclcpp::Parameter("size_x", 4.0),
    rclcpp::Parameter("size_y", 4.0),
    rclcpp::Parameter("origin_x", -2.0),
    rclcpp::Parameter("origin_y", -2.0),
    rclcpp::Parameter("publish_rate", 100.0),
    rclcpp::Parameter("cloud_stale_warning_age", 0.30),
  });
  auto flat_mapper = std::make_shared<TerrainMapperNode>(options);
  auto flat_cloud_pub = harness_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS());
  auto flat_odom_pub = harness_node_->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS());
  std::vector<utree_dog_msgs::msg::TerrainGrid> maps;
  std::vector<nav_msgs::msg::GridCells> raw_layers;
  std::vector<nav_msgs::msg::GridCells> inflated_layers;
  std::vector<nav_msgs::msg::OccupancyGrid> costmaps;
  std::vector<sensor_msgs::msg::PointCloud2> filtered_points;
  std::vector<sensor_msgs::msg::PointCloud2> filtered_maps;
  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  auto map_sub = harness_node_->create_subscription<utree_dog_msgs::msg::TerrainGrid>(
    namespace_name + "/terrain_map", map_qos,
    [&maps](const utree_dog_msgs::msg::TerrainGrid::SharedPtr msg) {maps.push_back(*msg);});
  auto raw_sub = harness_node_->create_subscription<nav_msgs::msg::GridCells>(
    namespace_name + "/flat_obstacle_raw", map_qos,
    [&raw_layers](const nav_msgs::msg::GridCells::SharedPtr msg) {raw_layers.push_back(*msg);});
  auto inflated_sub = harness_node_->create_subscription<nav_msgs::msg::GridCells>(
    namespace_name + "/flat_obstacle_inflated", map_qos,
    [&inflated_layers](const nav_msgs::msg::GridCells::SharedPtr msg) {
      inflated_layers.push_back(*msg);
    });
  auto cost_sub = harness_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    namespace_name + "/terrain_costmap", map_qos,
    [&costmaps](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {costmaps.push_back(*msg);});
  auto filtered_sub = harness_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    namespace_name + "/flat_obstacle_filtered_points", rclcpp::SensorDataQoS(),
    [&filtered_points](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_points.push_back(*msg);
    });
  auto filtered_map_sub = harness_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    namespace_name + "/flat_obstacle_filtered_map_3d",
    rclcpp::QoS(8).reliable().durability_volatile(),
    [&filtered_maps](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_maps.push_back(*msg);
    });
  executor_.add_node(flat_mapper);
  ASSERT_TRUE(spinUntil(
      [&flat_cloud_pub, &flat_odom_pub, &map_sub, &raw_sub, &inflated_sub, &cost_sub,
        &filtered_sub, &filtered_map_sub]() {
        return flat_cloud_pub->get_subscription_count() == 1U &&
               flat_odom_pub->get_subscription_count() == 1U &&
               map_sub->get_publisher_count() == 1U &&
               raw_sub->get_publisher_count() == 1U &&
               inflated_sub->get_publisher_count() == 1U &&
               cost_sub->get_publisher_count() == 1U &&
               filtered_sub->get_publisher_count() == 1U &&
               filtered_map_sub->get_publisher_count() == 1U;
      },
      1s));

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.z = 0.34;
  odom.pose.pose.orientation.w = 1.0;
  const std::vector<std::array<float, 3>> obstacle{{{0.80F, 0.30F, 0.12F}}};
  for (int frame = 0; frame < 2; ++frame) {
    spinFor(20ms);
    const builtin_interfaces::msg::Time stamp = harness_node_->now();
    odom.header.stamp = stamp;
    flat_odom_pub->publish(odom);
    flat_cloud_pub->publish(makeFlatSceneCloud(stamp, 0.0, 0.0, 0.0, obstacle));
  }
  ASSERT_TRUE(spinUntil(
      [&maps, &raw_layers, &costmaps, &filtered_points, &filtered_maps]() {
        return !maps.empty() && !maps.back().traversability.empty() &&
               !raw_layers.empty() && raw_layers.back().cells.size() == 1U &&
               !costmaps.empty() &&
               std::count(costmaps.back().data.begin(), costmaps.back().data.end(), 100) > 0 &&
               !filtered_points.empty() && filtered_points.back().width == 1U &&
               !filtered_maps.empty() && filtered_maps.back().width == 1U;
      },
      1s));

  ASSERT_TRUE(spinUntil(
      [&maps, &raw_layers, &inflated_layers, &costmaps, &filtered_points,
        &filtered_maps]() {
        return !maps.empty() && maps.back().traversability.empty() &&
               !raw_layers.empty() && raw_layers.back().cells.empty() &&
               !inflated_layers.empty() && inflated_layers.back().cells.empty() &&
               !costmaps.empty() && std::all_of(
          costmaps.back().data.begin(), costmaps.back().data.end(),
          [](std::int8_t value) {return value == -1;}) &&
               !filtered_points.empty() && filtered_points.back().width == 0U &&
               !filtered_maps.empty() && filtered_maps.back().width == 0U;
      },
      1s));

  executor_.remove_node(flat_mapper);
}

}  // namespace
}  // namespace utree_dog_navigation
