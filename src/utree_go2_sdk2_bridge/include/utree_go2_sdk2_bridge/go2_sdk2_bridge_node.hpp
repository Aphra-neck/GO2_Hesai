#pragma once

#include <cstdint>
#include <memory>
#include <optional>
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
// Motion is disarmed by default. One explicit arm authorizes subsequent valid paths,
// while every active command is stopped whenever its required input becomes stale.
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
  bool waitForNewPath(const char * reason);
  bool stopRobot(const char * reason) noexcept;
  bool cachedPathValid() const;
  bool cachedOdomValid() const;
  bool pathFresh(const rclcpp::Time & current_time) const;
  bool odomFresh(const rclcpp::Time & current_time) const;
  double messageAgeSeconds(
    const rclcpp::Time & message_time,
    const rclcpp::Time & current_time) const;
  bool messageStampFresh(
    const rclcpp::Time & message_time,
    const rclcpp::Time & current_time,
    double timeout) const;
  bool lowcmdPublisherPresent();

  std::string network_interface_;
  std::string world_frame_;
  std::string body_frame_;
  int domain_id_{0};
  MotionAuthorization motion_authorization_;
  // Cleared only after SportClient confirms StopMove.
  bool command_active_{false};
  double command_rate_{20.0};
  double path_timeout_{1.0};
  double odom_timeout_{0.5};
  double timestamp_future_tolerance_{0.2};
  double lookahead_distance_{0.6};
  double goal_position_tolerance_{0.15};
  double goal_yaw_tolerance_{0.20};
  double heading_alignment_enter_angle_{0.7853981633974483};
  double heading_alignment_exit_angle_{0.2617993877991494};
  bool heading_alignment_active_{false};
  double linear_gain_{1.0};
  double yaw_gain_{1.5};
  double max_vx_{};
  double max_vy_{};
  double max_yaw_rate_{};
  PathProgressTracker path_progress_tracker_;
  CompletedGoalLatch completed_goal_latch_;
  std::optional<std::int64_t> path_goal_generation_;
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
