#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "utree_dog_navigation/body_lattice_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utree_dog_navigation::BodyLatticePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
