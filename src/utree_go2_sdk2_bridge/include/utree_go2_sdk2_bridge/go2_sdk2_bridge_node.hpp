#pragma once

#include <chrono>
#include <cstddef>
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

namespace utree_go2_sdk2_bridge
{

// Official-style SDK2 path executor.
//
// The planner owns route generation and publishes /body_path. Once the
// operator arms this node, the control timer calls SportClient::Move directly
// on every tick. A completed route stays armed and emits Move(0, 0, 0) until
// a path for a new goal arrives. The bridge deliberately has no cross-track,
// progress, or "no motion" disarm gate: the only runtime stop paths are an
// SDK failure, an input timeout, a /lowcmd conflict, an explicit disable, or
// node shutdown.
class Go2Sdk2BridgeNode : public rclcpp::Node
{
public:
  Go2Sdk2BridgeNode();
  ~Go2Sdk2BridgeNode() noexcept override;

private:
  struct PathCommand
  {
    double vx{0.0};
    double vy{0.0};
    double yaw_rate{0.0};
    bool completed{false};
  };

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void enableCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void controlTick();
  void controlTickImpl();

  std::optional<PathCommand> makePathCommand();
  void reanchorPathCursor();
  void clearExecutionState();
  void disableAfterFault(const char * reason);
  bool sendMove(double vx, double vy, double yaw_rate);
  bool stopRobot(const char * reason, bool force = false) noexcept;
  bool cachedPathValid() const;
  bool cachedOdomValid() const;
  bool pathFresh() const;
  bool odomFresh() const;
  bool lowcmdPublisherPresent();

  std::string network_interface_;
  std::string world_frame_;
  std::string body_frame_;
  int domain_id_{0};

  double command_rate_{200.0};
  double path_timeout_{1.0};
  double odom_timeout_{0.5};
  double lookahead_distance_{0.35};
  std::size_t truncated_path_sample_count_{8U};
  double truncated_path_discount_{0.95};
  double waypoint_tolerance_{0.12};
  double goal_position_tolerance_{0.15};
  double heading_alignment_enter_angle_{0.35};
  double heading_alignment_exit_angle_{0.12};
  double persistent_arc_switch_angle_{0.04};
  double translation_speed_{0.20};
  double rotation_speed_{0.30};
  double max_vx_{0.6};
  double max_vy_{0.35};
  double max_yaw_rate_{0.8};

  bool motion_enabled_{false};
  bool command_active_{false};
  bool heading_alignment_active_{false};
  int persistent_arc_sign_{0};
  bool path_waiting_for_new_goal_{false};
  bool path_refresh_pending_reanchor_{false};

  std::size_t path_cursor_index_{0U};
  std::optional<std::int64_t> path_goal_generation_;
  std::optional<std::int64_t> completed_goal_generation_;
  nav_msgs::msg::Path::SharedPtr path_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  std::chrono::steady_clock::time_point last_path_received_{};
  std::chrono::steady_clock::time_point last_odom_received_{};

  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace utree_go2_sdk2_bridge
