#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "BRoCoManager.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BRoCoManager>());
  rclcpp::shutdown();
  return 0;
}