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

    // Load parameters from the specified YAML file
  this->declare_parameter("JETSON_NODE_ID");
  this->declare_parameter("SC_CONTAINER_NODE_ID");
  this->declare_parameter("SC_DRILL_NODE_ID");
  this->declare_parameter("NAV_NODE_ID");
  this->declare_parameter("HD_NODE_ID");
  this->declare_parameter("GENERAL_NODE_ID");
}

BRoCoManager::~BRoCoManager() {
    delete this->sub;
    delete this->pub;
    delete this->bus;
    delete this->driver;
}