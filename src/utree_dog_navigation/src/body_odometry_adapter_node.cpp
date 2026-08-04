#include "utree_dog_navigation/body_odometry_adapter_node.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

#include "utree_dog_navigation/body_odometry.hpp"

namespace utree_dog_navigation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

BodyOdometryAdapterNode::BodyOdometryAdapterNode() : Node("body_odom_adapter")
{
  input_topic_ = declare_parameter("input_odom_topic", "/lio/odom");
  output_topic_ = declare_parameter("output_odom_topic", "/lio/body_odom");
  world_frame_ = declare_parameter("world_frame", "world");
  body_frame_ = declare_parameter("body_frame", "base_link");
  yaw_offset_ = declare_parameter("yaw_offset", -0.5 * kPi);

  if (input_topic_.empty() || output_topic_.empty() || world_frame_.empty() ||
    body_frame_.empty())
  {
    throw std::invalid_argument("body odometry topics and frames must not be empty");
  }
  if (input_topic_ == output_topic_) {
    throw std::invalid_argument("input_odom_topic and output_odom_topic must differ");
  }
  if (!std::isfinite(yaw_offset_) || std::abs(yaw_offset_) > kPi) {
    throw std::invalid_argument("yaw_offset must be finite and in [-pi, pi] radians");
  }

  const auto qos = rclcpp::SensorDataQoS();
  body_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, qos);
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    input_topic_, qos,
    std::bind(&BodyOdometryAdapterNode::odometryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Body odometry %s -> %s: frame %s -> %s, yaw offset %.3f deg",
    input_topic_.c_str(), output_topic_.c_str(), world_frame_.c_str(), body_frame_.c_str(),
    yaw_offset_ * 180.0 / kPi);
}

void BodyOdometryAdapterNode::odometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_ERROR(get_logger(), "Rejected null input odometry");
    return;
  }
  nav_msgs::msg::Odometry output;
  const BodyOdometryStatus status = makeBodyOdometry(
    *msg, yaw_offset_, world_frame_, body_frame_, output);
  if (status != BodyOdometryStatus::kOk) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected input odometry: %s", bodyOdometryStatusMessage(status));
    return;
  }
  body_odometry_pub_->publish(output);
}

}  // namespace utree_dog_navigation
