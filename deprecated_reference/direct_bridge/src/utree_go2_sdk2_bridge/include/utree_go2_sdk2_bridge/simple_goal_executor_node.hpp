#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree/robot/go2/sport/sport_client.hpp"
#include "utree_go2_sdk2_bridge/simple_navigation_controller.hpp"

namespace utree_go2_sdk2_bridge
{

// Direct SDK2 bridge: a goal plus existing body odometry become a Manhattan
// route with heading alignment at each leg. The terrain stack may keep running,
// but its body path is not an input to this bridge.
class SimpleGoalExecutorNode : public rclcpp::Node
{
public:
  SimpleGoalExecutorNode();
  ~SimpleGoalExecutorNode() noexcept override;

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void enableCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void controlTick();
  void controlTickImpl();

  bool lowcmdPublisherPresent();
  bool odomFresh() const;
  bool finiteOdom() const;
  bool stopRobot(const char * reason) noexcept;
  bool sendMove(double vx, double vy, double yaw_rate);
  void clearGoal(const char * reason);
  std::optional<double> currentYaw() const;

  std::string network_interface_;
  std::string world_frame_;
  std::string body_frame_;
  std::string goal_topic_;
  std::string odom_topic_;
  std::string command_topic_;
  int domain_id_{0};

  double command_rate_{20.0};
  double odom_timeout_{1.0};
  double position_tolerance_{0.15};
  double yaw_tolerance_{0.12};
  double align_tolerance_{0.08};
  double waypoint_cross_track_tolerance_{0.30};
  double linear_gain_{1.0};
  double lateral_gain_{1.0};
  double yaw_gain_{1.5};
  double max_vx_{0.6};
  double max_vy_{0.35};
  double max_yaw_rate_{0.8};

  bool armed_{false};
  bool command_active_{false};
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::chrono::steady_clock::time_point last_odom_received_{};
  SimpleNavigationController navigation_;

  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace utree_go2_sdk2_bridge
