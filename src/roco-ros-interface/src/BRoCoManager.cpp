#include "rclcpp/rclcpp.hpp"

#include "BRoCoManager.h"

#include "BRoCo/CanSocketDriver.h"
#include "BRoCo/CANBus.h"


BRoCoManager::BRoCoManager() : Node("broco_manager") {
  this->driver = new CanSocketDriver("can0");
  this->bus = new CANBus(this->driver);
  this->pub = new BRoCoPublisher(this->bus, this);
  this->sub = new BRoCoSubscriber(this->bus, this->driver, this);
}
