#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree/robot/go2/sport/sport_client.hpp"
#include "utree_go2_sdk2_bridge/control_safety.hpp"

namespace utree_go2_sdk2_bridge
{

// Converts the geometric body path into bounded Go2 SportClient velocity commands.
// Motion is disabled by default and is stopped whenever required input becomes stale.
class Go2Sdk2BridgeNode : public rclcpp::Node
{
public:
  Go2Sdk2BridgeNode();
  ~Go2Sdk2BridgeNode() noexcept override;

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void enableCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void controlTick();
  void controlTickImpl();
  void failSafe(const char * reason);
  void stopRobot(const char * reason) noexcept;
  bool cachedInputsValid() const;
  bool inputsFresh(const rclcpp::Time & current_time) const;
  double messageAgeSeconds(
    const rclcpp::Time & message_time,
    const rclcpp::Time & current_time) const;
  bool messageStampFresh(
    const rclcpp::Time & message_time,
    const rclcpp::Time & current_time,
    double timeout) const;
  bool lowcmdPublisherPresent();
  std::size_t selectLookaheadPose() const;

  std::string network_interface_;
  std::string world_frame_;
  std::string body_frame_;
  int domain_id_{0};
  bool enabled_{false};
  // Cleared only after SportClient confirms StopMove.
  bool command_active_{false};
  double command_rate_{20.0};
  double path_timeout_{1.0};
  double odom_timeout_{0.5};
  double timestamp_future_tolerance_{0.2};
  double lookahead_distance_{0.6};
  double goal_position_tolerance_{0.15};
  double goal_yaw_tolerance_{0.20};
  double linear_gain_{1.0};
  double yaw_gain_{1.5};
  double max_vx_{0.6};
  double max_vy_{0.35};
  double max_yaw_rate_{0.8};
  nav_msgs::msg::Path::SharedPtr path_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace utree_go2_sdk2_bridge
