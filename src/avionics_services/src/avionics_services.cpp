/*
 * avionics_services.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"
#include "ServiceManager.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("avionics_services");
  RCLCPP_INFO(node->get_logger(), "Creating node...");

  rclcpp::spin(std::make_shared<ServiceManager>());
  rclcpp::shutdown();
  return 0;
}