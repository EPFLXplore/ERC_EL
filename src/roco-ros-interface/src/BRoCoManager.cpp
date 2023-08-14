/*
 * BRoCoManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "BRoCoManager.h"


BRoCoManager::BRoCoManager() : Node("broco_manager") {
  RCLCPP_DEBUG(this->get_logger(), "Creating publishers and subscribers...");
  this->driver = new CanSocketDriver("can0");
  // this->bus = new CANBus(this->driver);
  this->bus = new CANBus(this->driver);
  this->pub = new BRoCoPublisher(this->bus, this);
  this->sub = new BRoCoSubscriber(this->bus, this);
}

BRoCoManager::~BRoCoManager() {
    delete this->sub;
    delete this->pub;
    delete this->bus;
    delete this->driver;
}
