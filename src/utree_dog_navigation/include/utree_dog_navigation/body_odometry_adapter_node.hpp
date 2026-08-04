#pragma once

#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace utree_dog_navigation
{

class BodyOdometryAdapterNode : public rclcpp::Node
{
public:
  BodyOdometryAdapterNode();

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::string input_topic_;
  std::string output_topic_;
  std::string world_frame_;
  std::string body_frame_;
  double yaw_offset_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_odometry_pub_;
};

}  // namespace utree_dog_navigation
