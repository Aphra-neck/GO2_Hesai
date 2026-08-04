#include <memory>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "utree_go2_sdk2_bridge/go2_sdk2_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<utree_go2_sdk2_bridge::Go2Sdk2BridgeNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("go2_sdk2_bridge"), "Failed to start: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_sdk2_bridge"), "Failed with an unknown exception");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
