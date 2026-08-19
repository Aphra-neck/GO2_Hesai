#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
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
  void sportStateCallback(const void * message);
  void enableCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void controlTick();
  void controlTickImpl();
  void failSafe(
    const char * reason,
    JoystickRecoveryPolicy recovery_policy =
    JoystickRecoveryPolicy::kRequireConfirmedStop);
  bool waitForNewPath(const char * reason);
  bool suppressJoystickForSdkControl();
  bool stopRobot(
    const char * reason,
    JoystickRecoveryPolicy recovery_policy =
    JoystickRecoveryPolicy::kRequireConfirmedStop) noexcept;
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
  std::optional<SportStateSample> freshSportState();
  bool lowcmdPublisherPresent();

  std::string network_interface_;
  std::string world_frame_;
  std::string body_frame_;
  int domain_id_{0};
  MotionAuthorization motion_authorization_;
  SdkControlOwnership sdk_control_ownership_;
  PostJoystickSportStateGate post_joystick_sport_state_gate_;
  JoystickRecoveryPolicyLatch joystick_recovery_policy_latch_;
  double command_rate_{20.0};
  double path_timeout_{1.0};
  double odom_timeout_{0.5};
  double sport_state_timeout_{1.0};
  double balance_stand_timeout_{3.0};
  double balance_stand_retry_interval_{0.25};
  double timestamp_future_tolerance_{0.2};
  double lookahead_distance_{0.6};
  double goal_position_tolerance_{0.15};
  double goal_yaw_tolerance_{0.20};
  double heading_alignment_enter_angle_{0.7853981633974483};
  double heading_alignment_exit_angle_{0.2617993877991494};
  double explicit_rotation_tolerance_{0.05};
  bool heading_alignment_active_{false};
  bool balance_stand_pending_{false};
  std::chrono::steady_clock::time_point balance_stand_requested_at_{};
  std::chrono::steady_clock::time_point balance_stand_last_attempt_at_{};
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
  mutable std::mutex sport_state_mutex_;
  UnsafeSportStateLatch unsafe_sport_state_latch_;
  std::optional<std::uint32_t> sport_state_code_;
  std::uint8_t sport_state_mode_{0U};
  std::uint8_t sport_state_gait_type_{0U};
  std::uint64_t sport_state_sequence_{0U};
  std::chrono::steady_clock::time_point sport_state_received_at_{};
  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
  sport_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace utree_go2_sdk2_bridge
