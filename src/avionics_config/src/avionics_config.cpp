/*
 * avionics_config.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"
#include "ConfigManager.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("avionics_config");
  RCLCPP_INFO(node->get_logger(), "Creating node...");

  rclcpp::spin(std::make_shared<ConfigManager>());
  rclcpp::shutdown();
  return 0;
}