#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "utree_go2_sdk2_bridge/simple_goal_executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<utree_go2_sdk2_bridge::SimpleGoalExecutorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("go2_sdk2_direct_bridge"), "Failed to start: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_sdk2_direct_bridge"), "Failed with an unknown exception");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
