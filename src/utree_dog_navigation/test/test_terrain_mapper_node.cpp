#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
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

  std::size_t cellIndex(
    const utree_dog_msgs::msg::TerrainGrid & terrain, double world_x, double world_y) const
  {
    const auto grid_x = static_cast<std::size_t>(
      std::floor((world_x - terrain.origin_x) / terrain.resolution));
    const auto grid_y = static_cast<std::size_t>(
      std::floor((world_y - terrain.origin_y) / terrain.resolution));
    return grid_y * terrain.width + grid_x;
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

}  // namespace
}  // namespace utree_dog_navigation
