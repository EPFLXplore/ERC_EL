#include "rclcpp/rclcpp.hpp"

#include "BRoCoManager.h"

#include "BRoCo/CanSocketDriver.h"
#include "BRoCo/CANBus.h"


BRoCoManager::BRoCoManager() : Node("broco_manager") {
  RCLCPP_DEBUG(this->get_logger(), "Creating publishers and subscribers...");
  this->driver = new CanSocketDriver("can0");
  // this->bus = new CANBus(this->driver);
  this->bus = std::make_shared<CANBus>(this->driver);
  this->pub = new BRoCoPublisher(this->bus, this);
  this->sub = new BRoCoSubscriber(this->bus, this->driver, this);
}
