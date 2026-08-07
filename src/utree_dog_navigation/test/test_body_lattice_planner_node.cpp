#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utree_dog_msgs/msg/terrain_grid.hpp"
#include "utree_dog_navigation/body_lattice_planner_node.hpp"

namespace utree_dog_navigation
{
namespace
{

using namespace std::chrono_literals;

class BodyLatticePlannerNodeTest : public ::testing::Test
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
    map_topic_ = "/planner_safety_test_" + suffix + "/terrain_map";
    odom_topic_ = "/planner_safety_test_" + suffix + "/body_odom";
    goal_topic_ = "/planner_safety_test_" + suffix + "/goal";
    path_topic_ = "/planner_safety_test_" + suffix + "/body_path";

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("terrain_map_topic", map_topic_),
      rclcpp::Parameter("odom_topic", odom_topic_),
      rclcpp::Parameter("goal_topic", goal_topic_),
      rclcpp::Parameter("path_topic", path_topic_),
      rclcpp::Parameter("map_frame", "world"),
      rclcpp::Parameter("body_frame", "base_link"),
      rclcpp::Parameter("max_map_age", 1.0),
      rclcpp::Parameter("max_odom_age", 1.0),
      rclcpp::Parameter("timestamp_future_tolerance", 0.05),
      rclcpp::Parameter("input_watchdog_rate", 50.0),
    });
    planner_node_ = std::make_shared<BodyLatticePlannerNode>(options);
    harness_node_ = std::make_shared<rclcpp::Node>("planner_safety_harness_" + suffix);

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = harness_node_->create_publisher<utree_dog_msgs::msg::TerrainGrid>(
      map_topic_, map_qos);
    odom_pub_ = harness_node_->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS());
    goal_pub_ = harness_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10));
    path_sub_ = harness_node_->create_subscription<nav_msgs::msg::Path>(
      path_topic_, map_qos,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {paths_.push_back(*msg);});

    executor_.add_node(planner_node_);
    executor_.add_node(harness_node_);
    ASSERT_TRUE(spinUntil(
        [this]() {
          return map_pub_->get_subscription_count() == 1U &&
                 odom_pub_->get_subscription_count() == 1U &&
                 goal_pub_->get_subscription_count() == 1U &&
                 path_sub_->get_publisher_count() == 1U;
        },
        2s));
  }

  void TearDown() override
  {
    executor_.remove_node(harness_node_);
    executor_.remove_node(planner_node_);
    path_sub_.reset();
    goal_pub_.reset();
    odom_pub_.reset();
    map_pub_.reset();
    harness_node_.reset();
    planner_node_.reset();
    paths_.clear();
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

  utree_dog_msgs::msg::TerrainGrid makeMap(
    const rclcpp::Time & stamp, bool traversable = true) const
  {
    utree_dog_msgs::msg::TerrainGrid map;
    map.header.stamp = stamp;
    map.header.frame_id = "world";
    map.resolution = 0.2F;
    map.width = 10;
    map.height = 10;
    map.origin_x = 0.0F;
    map.origin_y = 0.0F;
    map.unknown_value = -1000.0F;
    const std::size_t cell_count = map.width * map.height;
    const float value = traversable ? 0.0F : map.unknown_value;
    map.elevation.assign(cell_count, value);
    map.slope.assign(cell_count, value);
    map.traversability.assign(cell_count, traversable ? 1.0F : map.unknown_value);
    return map;
  }

  nav_msgs::msg::Odometry makeOdom(const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 0.3;
    odom.pose.pose.position.y = 0.3;
    odom.pose.pose.orientation.w = 1.0;
    return odom;
  }

  geometry_msgs::msg::PoseStamped makeGoal(const rclcpp::Time & stamp) const
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = "world";
    goal.pose.position.x = 1.3;
    goal.pose.position.y = 0.3;
    goal.pose.orientation.w = 1.0;
    return goal;
  }

  builtin_interfaces::msg::Time publishFreshPlan()
  {
    const auto current_time = harness_node_->now();
    const auto map_time = current_time - 50ms;
    map_pub_->publish(makeMap(map_time));
    odom_pub_->publish(makeOdom(current_time - 20ms));
    goal_pub_->publish(makeGoal(current_time));
    EXPECT_TRUE(spinUntil(
        [this]() {return !paths_.empty() && !paths_.back().poses.empty();}, 2s));
    return map_time;
  }

  inline static int instance_count_{0};
  std::string map_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<BodyLatticePlannerNode> planner_node_;
  rclcpp::Node::SharedPtr harness_node_;
  rclcpp::Publisher<utree_dog_msgs::msg::TerrainGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::vector<nav_msgs::msg::Path> paths_;
};

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathWhenMapIsStale)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  const auto current_time = harness_node_->now();
  odom_pub_->publish(makeOdom(current_time));
  map_pub_->publish(makeMap(current_time - 2s));

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {return paths_.size() > messages_before_failure;}, 1s));
  EXPECT_TRUE(paths_.back().poses.empty());
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathWhenMapStampIsFromFuture)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  const auto current_time = harness_node_->now();
  odom_pub_->publish(makeOdom(current_time));
  map_pub_->publish(makeMap(current_time + 1s));

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathForNegativeMapTimestampWithoutThrowing)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto map = makeMap(harness_node_->now());
  map.header.stamp.sec = -1;
  map.header.stamp.nanosec = 0U;
  map_pub_->publish(map);

  bool path_cleared = false;
  EXPECT_NO_THROW(
    path_cleared = spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
  EXPECT_TRUE(path_cleared);
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathForInvalidOdometryNanosecondsWithoutThrowing)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto odom = makeOdom(harness_node_->now());
  ASSERT_GT(odom.header.stamp.sec, 0);
  --odom.header.stamp.sec;
  odom.header.stamp.nanosec = 1000000000U;
  odom_pub_->publish(odom);

  bool path_cleared = false;
  EXPECT_NO_THROW(
    path_cleared = spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 300ms));
  EXPECT_TRUE(path_cleared);
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathForZeroMapTimestampWithoutThrowing)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto map = makeMap(harness_node_->now());
  map.header.stamp.sec = 0;
  map.header.stamp.nanosec = 0U;
  map_pub_->publish(map);

  bool path_cleared = false;
  EXPECT_NO_THROW(
    path_cleared = spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
  EXPECT_TRUE(path_cleared);
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathWhenOdometryIsStale)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  const auto current_time = harness_node_->now();
  odom_pub_->publish(makeOdom(current_time - 2s));
  map_pub_->publish(makeMap(current_time));

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
  EXPECT_TRUE(paths_.back().poses.empty());
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathForMalformedTerrainMap)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto map = makeMap(harness_node_->now());
  map.elevation[0] = std::numeric_limits<float>::quiet_NaN();
  map_pub_->publish(map);

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathForNonFiniteOdometryPose)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto odom = makeOdom(harness_node_->now());
  odom.pose.pose.position.x = std::numeric_limits<double>::infinity();
  odom_pub_->publish(odom);

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathForNonFiniteGoalPose)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto goal = makeGoal(harness_node_->now());
  goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  goal_pub_->publish(goal);

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathForInvalidGoalQuaternion)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto goal = makeGoal(harness_node_->now());
  goal.pose.orientation.w = 2.0;
  goal_pub_->publish(goal);

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, RejectsMutuallyConsistentInputsOutsideConfiguredMapFrame)
{
  const auto current_time = harness_node_->now();
  auto map = makeMap(current_time);
  map.header.frame_id = "map";
  auto odom = makeOdom(current_time);
  odom.header.frame_id = "map";
  auto goal = makeGoal(current_time);
  goal.header.frame_id = "map";

  map_pub_->publish(map);
  odom_pub_->publish(odom);
  goal_pub_->publish(goal);
  spinFor(300ms);

  EXPECT_TRUE(paths_.empty());
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathWhenOdometryMapFrameChanges)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto odom = makeOdom(harness_node_->now());
  odom.header.frame_id = "map";
  odom_pub_->publish(odom);

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, ClearsAnActivePathWhenOdometryBodyFrameChanges)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  auto odom = makeOdom(harness_node_->now());
  odom.child_frame_id = "imu";
  odom_pub_->publish(odom);

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {
        return paths_.size() > messages_before_failure && paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, RejectsWatchdogPeriodLongerThanFreshnessBudget)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("max_map_age", 0.5),
    rclcpp::Parameter("max_odom_age", 0.1),
    rclcpp::Parameter("input_watchdog_rate", 5.0),
  });

  EXPECT_THROW(
    std::make_shared<BodyLatticePlannerNode>(options),
    std::invalid_argument);
}

TEST_F(BodyLatticePlannerNodeTest, WatchdogClearsAPathWhenInputsStop)
{
  publishFreshPlan();
  const std::size_t messages_before_timeout = paths_.size();

  ASSERT_TRUE(spinUntil(
      [this, messages_before_timeout]() {return paths_.size() > messages_before_timeout;}, 1500ms));
  EXPECT_TRUE(paths_.back().poses.empty());
}

TEST_F(BodyLatticePlannerNodeTest, PathTimestampPreservesOldestCausalInput)
{
  const auto expected_stamp = publishFreshPlan();

  EXPECT_EQ(
    rclcpp::Time(paths_.back().header.stamp).nanoseconds(),
    rclcpp::Time(expected_stamp).nanoseconds());
}

TEST_F(BodyLatticePlannerNodeTest, PlanningFailureReplacesTransientLocalPathWithEmptyPath)
{
  publishFreshPlan();
  const std::size_t messages_before_failure = paths_.size();
  const auto current_time = harness_node_->now();
  odom_pub_->publish(makeOdom(current_time));
  map_pub_->publish(makeMap(current_time, false));

  ASSERT_TRUE(spinUntil(
      [this, messages_before_failure]() {return paths_.size() > messages_before_failure;}, 1s));
  ASSERT_TRUE(paths_.back().poses.empty());

  map_pub_->publish(makeMap(current_time, false));
  map_pub_->publish(makeMap(current_time, false));
  spinFor(200ms);
  const auto empty_messages = std::count_if(
    paths_.begin() + static_cast<std::ptrdiff_t>(messages_before_failure), paths_.end(),
    [](const nav_msgs::msg::Path & path) {return path.poses.empty();});
  EXPECT_EQ(empty_messages, 1);

  std::vector<nav_msgs::msg::Path> late_paths;
  const auto late_sub = harness_node_->create_subscription<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(1).reliable().transient_local(),
    [&late_paths](const nav_msgs::msg::Path::SharedPtr msg) {late_paths.push_back(*msg);});
  ASSERT_TRUE(spinUntil([&late_paths]() {return !late_paths.empty();}, 1s));
  EXPECT_TRUE(late_paths.back().poses.empty());
}

}  // namespace
}  // namespace utree_dog_navigation
