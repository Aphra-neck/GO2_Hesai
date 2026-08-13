#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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
      rclcpp::Parameter("planning_mode", planning_mode_),
      rclcpp::Parameter("flat_ground_confirmed", flat_ground_confirmed_),
      rclcpp::Parameter("max_map_age", 1.0),
      rclcpp::Parameter("max_odom_age", 1.0),
      rclcpp::Parameter("max_goal_age", max_goal_age_),
      rclcpp::Parameter("goal_retention_timeout", goal_retention_timeout_),
      rclcpp::Parameter("timestamp_future_tolerance", 0.05),
      rclcpp::Parameter("input_watchdog_rate", 50.0),
      rclcpp::Parameter("verified_flat_start.enabled", true),
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

  utree_dog_msgs::msg::TerrainGrid makeStandingBlindRingMap(
    const rclcpp::Time & stamp) const
  {
    utree_dog_msgs::msg::TerrainGrid map;
    map.header.stamp = stamp;
    map.header.frame_id = "world";
    map.resolution = 0.2F;
    map.width = 31;
    map.height = 31;
    map.origin_x = -3.1F;
    map.origin_y = -3.1F;
    map.unknown_value = -1000.0F;
    const std::size_t cell_count = map.width * map.height;
    map.elevation.assign(cell_count, map.unknown_value);
    map.slope.assign(cell_count, map.unknown_value);
    map.traversability.assign(cell_count, map.unknown_value);
    map.observation_count.assign(cell_count, 0U);
    for (std::size_t y = 0; y < map.height; ++y) {
      for (std::size_t x = 0; x < map.width; ++x) {
        const double world_x = map.origin_x + (static_cast<double>(x) + 0.5) * map.resolution;
        const double world_y = map.origin_y + (static_cast<double>(y) + 0.5) * map.resolution;
        const double radius = std::hypot(world_x, world_y);
        if (radius < 1.2 || radius > 2.4) {continue;}
        const std::size_t index = y * map.width + x;
        map.elevation[index] = 0.1F;
        map.slope[index] = 0.0F;
        map.traversability[index] = 1.0F;
        map.observation_count[index] = 4U;
      }
    }
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
  double max_goal_age_{2.0};
  double goal_retention_timeout_{30.0};
  std::string planning_mode_{"terrain"};
  bool flat_ground_confirmed_{false};
};

class ShortGoalRetentionPlannerNodeTest : public BodyLatticePlannerNodeTest
{
public:
  ShortGoalRetentionPlannerNodeTest()
  {
    goal_retention_timeout_ = 0.10;
  }
};

class FlatObstaclePlannerNodeTest : public BodyLatticePlannerNodeTest
{
public:
  FlatObstaclePlannerNodeTest()
  {
    planning_mode_ = "flat_obstacle";
    flat_ground_confirmed_ = true;
  }

protected:
  utree_dog_msgs::msg::TerrainGrid makeFlatMap(const rclcpp::Time & stamp) const
  {
    auto map = makeMap(stamp);
    map.width = 40;
    map.height = 40;
    map.origin_x = -4.0F;
    map.origin_y = -4.0F;
    const std::size_t cell_count = map.width * map.height;
    map.elevation.assign(cell_count, 0.0F);
    map.slope.assign(cell_count, 0.0F);
    map.traversability.assign(cell_count, 1.0F);
    return map;
  }

  builtin_interfaces::msg::Time publishFlatPlan(double body_z)
  {
    const auto current_time = harness_node_->now();
    const auto map_time = current_time - 50ms;
    auto odom = makeOdom(current_time - 20ms);
    odom.pose.pose.position.z = body_z;
    map_pub_->publish(makeFlatMap(map_time));
    odom_pub_->publish(odom);
    goal_pub_->publish(makeGoal(current_time));
    EXPECT_TRUE(spinUntil(
        [this]() {return !paths_.empty() && !paths_.back().poses.empty();}, 2s));
    return map_time;
  }
};

TEST_F(BodyLatticePlannerNodeTest, RejectsUnconfirmedFlatObstacleMode)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat_obstacle"),
    rclcpp::Parameter("flat_ground_confirmed", false),
  });

  EXPECT_THROW(
    std::make_shared<BodyLatticePlannerNode>(options),
    std::invalid_argument);
}

TEST_F(BodyLatticePlannerNodeTest, RejectsUnknownPlanningMode)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("planning_mode", "flat"),
  });

  EXPECT_THROW(
    std::make_shared<BodyLatticePlannerNode>(options),
    std::invalid_argument);
}

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

TEST_F(BodyLatticePlannerNodeTest, ClearsAStaleIncomingGoalAndRequiresANewGoal)
{
  publishFreshPlan();
  const std::size_t messages_before_rejection = paths_.size();
  goal_pub_->publish(makeGoal(harness_node_->now() - 3s));

  ASSERT_TRUE(spinUntil(
      [this, messages_before_rejection]() {
        return paths_.size() > messages_before_rejection && paths_.back().poses.empty();
      }, 1s));
  const std::size_t messages_after_rejection = paths_.size();

  const auto current_time = harness_node_->now();
  odom_pub_->publish(makeOdom(current_time));
  map_pub_->publish(makeMap(current_time));
  spinFor(200ms);
  EXPECT_EQ(paths_.size(), messages_after_rejection);

  goal_pub_->publish(makeGoal(harness_node_->now()));
  ASSERT_TRUE(spinUntil(
      [this, messages_after_rejection]() {
        return paths_.size() > messages_after_rejection && !paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, RejectsZeroTimestampGoalAndClearsTheActivePath)
{
  publishFreshPlan();
  const std::size_t messages_before_rejection = paths_.size();
  auto goal = makeGoal(harness_node_->now());
  goal.header.stamp.sec = 0;
  goal.header.stamp.nanosec = 0U;
  goal_pub_->publish(goal);

  ASSERT_TRUE(spinUntil(
      [this, messages_before_rejection]() {
        return paths_.size() > messages_before_rejection && paths_.back().poses.empty();
      }, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, RejectsFutureGoalAndClearsTheActivePath)
{
  publishFreshPlan();
  const std::size_t messages_before_rejection = paths_.size();
  goal_pub_->publish(makeGoal(harness_node_->now() + 1s));

  ASSERT_TRUE(spinUntil(
      [this, messages_before_rejection]() {
        return paths_.size() > messages_before_rejection && paths_.back().poses.empty();
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

TEST_F(BodyLatticePlannerNodeTest, RejectsInvalidGoalFreshnessBudgets)
{
  rclcpp::NodeOptions stale_header_options;
  stale_header_options.parameter_overrides({
    rclcpp::Parameter("max_goal_age", 0.0),
  });
  EXPECT_THROW(
    std::make_shared<BodyLatticePlannerNode>(stale_header_options),
    std::invalid_argument);

  rclcpp::NodeOptions retention_options;
  retention_options.parameter_overrides({
    rclcpp::Parameter(
      "goal_retention_timeout", std::numeric_limits<double>::quiet_NaN()),
  });
  EXPECT_THROW(
    std::make_shared<BodyLatticePlannerNode>(retention_options),
    std::invalid_argument);
}

TEST_F(BodyLatticePlannerNodeTest, RejectsNonFiniteNominalBodyHeight)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("nominal_body_height", std::numeric_limits<double>::quiet_NaN()),
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

TEST_F(
  ShortGoalRetentionPlannerNodeTest,
  ClearsExpiredGoalAndDoesNotRetryItOnFreshMap)
{
  publishFreshPlan();
  const std::size_t messages_before_timeout = paths_.size();

  ASSERT_TRUE(spinUntil(
      [this, messages_before_timeout]() {
        return paths_.size() > messages_before_timeout && paths_.back().poses.empty();
      }, 400ms));
  const std::size_t messages_after_timeout = paths_.size();

  const auto current_time = harness_node_->now();
  odom_pub_->publish(makeOdom(current_time));
  map_pub_->publish(makeMap(current_time));
  spinFor(200ms);
  EXPECT_EQ(paths_.size(), messages_after_timeout);

  goal_pub_->publish(makeGoal(harness_node_->now()));
  ASSERT_TRUE(spinUntil(
      [this, messages_after_timeout]() {
        return paths_.size() > messages_after_timeout && !paths_.back().poses.empty();
      }, 1s));
}

TEST_F(
  ShortGoalRetentionPlannerNodeTest,
  ExpiresACachedGoalBeforeAnyPathExists)
{
  goal_pub_->publish(makeGoal(harness_node_->now()));
  spinFor(250ms);

  const auto current_time = harness_node_->now();
  odom_pub_->publish(makeOdom(current_time));
  map_pub_->publish(makeMap(current_time));
  spinFor(200ms);
  EXPECT_TRUE(paths_.empty());

  goal_pub_->publish(makeGoal(harness_node_->now()));
  ASSERT_TRUE(spinUntil(
      [this]() {return !paths_.empty() && !paths_.back().poses.empty();}, 1s));
}

TEST_F(BodyLatticePlannerNodeTest, PathTimestampPreservesOldestCausalInput)
{
  const auto expected_stamp = publishFreshPlan();

  EXPECT_EQ(
    rclcpp::Time(paths_.back().header.stamp).nanoseconds(),
    rclcpp::Time(expected_stamp).nanoseconds());
}

TEST_F(BodyLatticePlannerNodeTest, PathPosesCarryStableGoalGenerationAcrossMapReplans)
{
  const auto initial_time = harness_node_->now();
  const auto initial_map_time = initial_time - 50ms;
  const auto initial_odom_time = initial_time - 20ms;
  const auto first_goal_time = initial_time - 10ms;

  map_pub_->publish(makeMap(initial_map_time));
  odom_pub_->publish(makeOdom(initial_odom_time));
  goal_pub_->publish(makeGoal(first_goal_time));
  ASSERT_TRUE(
    spinUntil(
      [this]() {return !paths_.empty() && !paths_.back().poses.empty();}, 2s));

  const auto expect_pose_generation = [](const nav_msgs::msg::Path & path,
      const rclcpp::Time & expected_generation) {
      ASSERT_FALSE(path.poses.empty());
      for (const auto & pose : path.poses) {
        EXPECT_EQ(pose.header.frame_id, path.header.frame_id);
        EXPECT_EQ(
          rclcpp::Time(pose.header.stamp).nanoseconds(),
          expected_generation.nanoseconds());
      }
    };

  EXPECT_EQ(
    rclcpp::Time(paths_.back().header.stamp).nanoseconds(),
    initial_map_time.nanoseconds());
  expect_pose_generation(paths_.back(), first_goal_time);

  const std::size_t paths_before_replan = paths_.size();
  const auto replan_map_time = harness_node_->now();
  map_pub_->publish(makeMap(replan_map_time));
  ASSERT_TRUE(
    spinUntil(
      [this, paths_before_replan]() {
        return paths_.size() > paths_before_replan && !paths_.back().poses.empty();
      }, 2s));

  EXPECT_EQ(
    rclcpp::Time(paths_.back().header.stamp).nanoseconds(),
    initial_odom_time.nanoseconds());
  expect_pose_generation(paths_.back(), first_goal_time);

  const std::size_t paths_before_new_goal = paths_.size();
  const auto second_goal_time = harness_node_->now();
  ASSERT_NE(second_goal_time.nanoseconds(), first_goal_time.nanoseconds());
  auto second_goal = makeGoal(second_goal_time);
  second_goal.pose.position.y = 0.5;
  goal_pub_->publish(second_goal);
  ASSERT_TRUE(
    spinUntil(
      [this, paths_before_new_goal]() {
        return paths_.size() > paths_before_new_goal && !paths_.back().poses.empty();
      }, 2s));

  EXPECT_EQ(
    rclcpp::Time(paths_.back().header.stamp).nanoseconds(),
    initial_odom_time.nanoseconds());
  expect_pose_generation(paths_.back(), second_goal_time);
  EXPECT_NE(
    rclcpp::Time(paths_.back().poses.front().header.stamp).nanoseconds(),
    first_goal_time.nanoseconds());
}

TEST_F(BodyLatticePlannerNodeTest, ObservedFlatPathRetainsHeightAndUnitOrientation)
{
  publishFreshPlan();

  ASSERT_FALSE(paths_.back().poses.empty());
  for (const auto & pose : paths_.back().poses) {
    EXPECT_NEAR(pose.pose.position.z, 0.42, 1.0e-9);
    const double orientation_norm =
      std::sqrt(
      pose.pose.orientation.x * pose.pose.orientation.x +
      pose.pose.orientation.y * pose.pose.orientation.y +
      pose.pose.orientation.z * pose.pose.orientation.z +
      pose.pose.orientation.w * pose.pose.orientation.w);
    EXPECT_NEAR(orientation_norm, 1.0, 1.0e-9);
  }
}

TEST_F(BodyLatticePlannerNodeTest, VerifiedFlatStartPublishesFinitePathFromExactOdometry)
{
  const auto current_time = harness_node_->now();
  auto odom = makeOdom(current_time - 20ms);
  odom.pose.pose.position.x = 0.03;
  odom.pose.pose.position.y = -0.04;
  auto goal = makeGoal(current_time);
  goal.pose.position.x = 1.4;
  goal.pose.position.y = 0.0;

  map_pub_->publish(makeStandingBlindRingMap(current_time - 50ms));
  odom_pub_->publish(odom);
  goal_pub_->publish(goal);

  ASSERT_TRUE(spinUntil(
      [this]() {return !paths_.empty() && !paths_.back().poses.empty();}, 2s));
  const auto & path = paths_.back();
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.front().pose.position.x, odom.pose.pose.position.x, 1.0e-9);
  EXPECT_NEAR(path.poses.front().pose.position.y, odom.pose.pose.position.y, 1.0e-9);
  EXPECT_NEAR(path.poses.front().pose.position.z, 0.52, 1.0e-5);
  for (const auto & pose : path.poses) {
    EXPECT_TRUE(std::isfinite(pose.pose.position.x));
    EXPECT_TRUE(std::isfinite(pose.pose.position.y));
    EXPECT_TRUE(std::isfinite(pose.pose.position.z));
    EXPECT_TRUE(std::isfinite(pose.pose.orientation.x));
    EXPECT_TRUE(std::isfinite(pose.pose.orientation.y));
    EXPECT_TRUE(std::isfinite(pose.pose.orientation.z));
    EXPECT_TRUE(std::isfinite(pose.pose.orientation.w));
    const double orientation_norm =
      std::sqrt(
      pose.pose.orientation.x * pose.pose.orientation.x +
      pose.pose.orientation.y * pose.pose.orientation.y +
      pose.pose.orientation.z * pose.pose.orientation.z +
      pose.pose.orientation.w * pose.pose.orientation.w);
    EXPECT_NEAR(orientation_norm, 1.0, 1.0e-9);
  }
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

TEST_F(FlatObstaclePlannerNodeTest, LocksEveryPathPoseToInitialStandingBodyHeight)
{
  constexpr double kInitialBodyZ = 0.037;
  publishFlatPlan(kInitialBodyZ);

  ASSERT_FALSE(paths_.back().poses.empty());
  for (const auto & pose : paths_.back().poses) {
    EXPECT_NEAR(pose.pose.position.z, kInitialBodyZ, 1.0e-9);
    EXPECT_NEAR(pose.pose.orientation.x, 0.0, 1.0e-9);
    EXPECT_NEAR(pose.pose.orientation.y, 0.0, 1.0e-9);
  }

  const std::size_t paths_before_drift = paths_.size();
  auto drifted_odom = makeOdom(harness_node_->now());
  drifted_odom.pose.pose.position.z = 0.41;
  odom_pub_->publish(drifted_odom);
  spinFor(50ms);
  map_pub_->publish(makeFlatMap(harness_node_->now()));
  ASSERT_TRUE(spinUntil(
      [this, paths_before_drift]() {return paths_.size() > paths_before_drift;}, 1s));
  ASSERT_FALSE(paths_.back().poses.empty());
  for (const auto & pose : paths_.back().poses) {
    EXPECT_NEAR(pose.pose.position.z, kInitialBodyZ, 1.0e-9);
  }
}

TEST_F(FlatObstaclePlannerNodeTest, MalformedEpochInvalidationRelocksStandingHeight)
{
  publishFlatPlan(0.03);
  const std::size_t paths_before_invalidation = paths_.size();

  utree_dog_msgs::msg::TerrainGrid invalidation;
  invalidation.header.stamp = harness_node_->now();
  invalidation.header.frame_id = "world";
  map_pub_->publish(invalidation);
  ASSERT_TRUE(spinUntil(
      [this, paths_before_invalidation]() {
        return paths_.size() > paths_before_invalidation && paths_.back().poses.empty();
      },
      1s));

  auto relocked_odom = makeOdom(harness_node_->now());
  relocked_odom.pose.pose.position.z = 1.03;
  odom_pub_->publish(relocked_odom);
  map_pub_->publish(makeFlatMap(harness_node_->now()));
  goal_pub_->publish(makeGoal(harness_node_->now()));
  ASSERT_TRUE(spinUntil(
      [this]() {return !paths_.empty() && !paths_.back().poses.empty();}, 2s));
  for (const auto & pose : paths_.back().poses) {
    EXPECT_NEAR(pose.pose.position.z, 1.03, 1.0e-9);
  }
}

TEST_F(FlatObstaclePlannerNodeTest, RetainsQuantizedStartPoseWhenExactYawDiffers)
{
  constexpr double kExactYaw = 0.10;
  const auto current_time = harness_node_->now();
  auto odom = makeOdom(current_time - 20ms);
  odom.pose.pose.orientation.z = std::sin(kExactYaw * 0.5);
  odom.pose.pose.orientation.w = std::cos(kExactYaw * 0.5);

  map_pub_->publish(makeFlatMap(current_time - 50ms));
  odom_pub_->publish(odom);
  goal_pub_->publish(makeGoal(current_time));

  ASSERT_TRUE(spinUntil(
      [this]() {return !paths_.empty() && !paths_.back().poses.empty();}, 2s));
  const auto & poses = paths_.back().poses;
  ASSERT_GE(poses.size(), 2U);
  EXPECT_NEAR(poses[0].pose.position.x, odom.pose.pose.position.x, 1.0e-9);
  EXPECT_NEAR(poses[0].pose.position.y, odom.pose.pose.position.y, 1.0e-9);
  EXPECT_NEAR(poses[1].pose.position.x, odom.pose.pose.position.x, 1.0e-6);
  EXPECT_NEAR(poses[1].pose.position.y, odom.pose.pose.position.y, 1.0e-6);
  EXPECT_NEAR(
    2.0 * std::atan2(poses[0].pose.orientation.z, poses[0].pose.orientation.w),
    kExactYaw, 1.0e-9);
  EXPECT_NEAR(
    2.0 * std::atan2(poses[1].pose.orientation.z, poses[1].pose.orientation.w),
    0.0, 1.0e-9);
}

TEST_F(FlatObstaclePlannerNodeTest, PublishesExecutableStartConnectorBeforeLatticePath)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kExactYaw = -91.87 * kPi / 180.0;
  const auto current_time = harness_node_->now();
  const auto source_time = current_time - 20ms;
  auto map = makeFlatMap(source_time);
  map.width = 200;
  map.height = 200;
  map.origin_x = -20.2F;
  map.origin_y = -20.0F;
  map.traversability.assign(
    static_cast<std::size_t>(map.width) * map.height, map.unknown_value);
  map.traversability[103U * map.width + 98U] = 0.0F;

  auto odom = makeOdom(source_time);
  odom.pose.pose.position.x = -0.032;
  odom.pose.pose.position.y = 0.025;
  odom.pose.pose.orientation.z = std::sin(kExactYaw * 0.5);
  odom.pose.pose.orientation.w = std::cos(kExactYaw * 0.5);

  auto goal = makeGoal(current_time);
  goal.pose.position.x = -0.1;
  goal.pose.position.y = -0.5;
  goal.pose.orientation.z = std::sin(-0.25 * kPi);
  goal.pose.orientation.w = std::cos(-0.25 * kPi);

  map_pub_->publish(map);
  odom_pub_->publish(odom);
  goal_pub_->publish(goal);

  ASSERT_TRUE(spinUntil(
      [this]() {return !paths_.empty() && !paths_.back().poses.empty();}, 2s));
  const auto & poses = paths_.back().poses;
  ASSERT_EQ(poses.size(), 5U);
  EXPECT_NEAR(poses[0].pose.position.x, odom.pose.pose.position.x, 1.0e-9);
  EXPECT_NEAR(poses[0].pose.position.y, odom.pose.pose.position.y, 1.0e-9);
  EXPECT_NEAR(poses[1].pose.position.x, -0.1, 1.0e-6);
  EXPECT_NEAR(poses[1].pose.position.y, -0.1, 1.0e-6);
  EXPECT_NEAR(poses[2].pose.position.x, poses[1].pose.position.x, 1.0e-9);
  EXPECT_NEAR(poses[2].pose.position.y, poses[1].pose.position.y, 1.0e-9);
  EXPECT_NEAR(
    2.0 * std::atan2(poses[1].pose.orientation.z, poses[1].pose.orientation.w),
    kExactYaw, 1.0e-9);
  EXPECT_NEAR(
    2.0 * std::atan2(poses[2].pose.orientation.z, poses[2].pose.orientation.w),
    -0.5 * kPi, 1.0e-9);
  EXPECT_NEAR(poses[3].pose.position.x, -0.1, 1.0e-6);
  EXPECT_NEAR(poses[3].pose.position.y, -0.3, 1.0e-6);
  EXPECT_NEAR(poses.back().pose.position.x, goal.pose.position.x, 1.0e-6);
  EXPECT_NEAR(poses.back().pose.position.y, goal.pose.position.y, 1.0e-6);
}

TEST_F(FlatObstaclePlannerNodeTest, StaleInputClearsCachedGoalAndRequiresNewGoal)
{
  publishFlatPlan(0.01);
  const std::size_t messages_before_stale = paths_.size();
  const auto current_time = harness_node_->now();
  odom_pub_->publish(makeOdom(current_time));
  map_pub_->publish(makeFlatMap(current_time - 2s));

  ASSERT_TRUE(spinUntil(
      [this, messages_before_stale]() {
        return paths_.size() > messages_before_stale && paths_.back().poses.empty();
      }, 1s));
  const std::size_t messages_after_stale = paths_.size();

  odom_pub_->publish(makeOdom(harness_node_->now()));
  map_pub_->publish(makeFlatMap(harness_node_->now()));
  spinFor(200ms);
  EXPECT_EQ(paths_.size(), messages_after_stale);

  goal_pub_->publish(makeGoal(harness_node_->now()));
  ASSERT_TRUE(spinUntil(
      [this, messages_after_stale]() {
        return paths_.size() > messages_after_stale && !paths_.back().poses.empty();
      }, 1s));
}

}  // namespace
}  // namespace utree_dog_navigation
