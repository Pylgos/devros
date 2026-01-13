#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("standalone_node");
  RCLCPP_INFO(node->get_logger(), "Standalone package E node started!");
  rclcpp::shutdown();
  return 0;
}
