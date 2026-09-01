#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "utree_dog_navigation/body_odometry_adapter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utree_dog_navigation::BodyOdometryAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
