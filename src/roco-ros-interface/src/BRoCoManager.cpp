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

  RCLCPP_INFO(this->get_logger(), "JETSON_NODE_ID: %d", get_param<uint32_t>("JETSON_NODE_ID"));

}

BRoCoManager::~BRoCoManager() {
    delete this->sub;
    delete this->pub;
    delete this->bus;
    delete this->driver;
}

template <typename T>
T BRoCoManager::get_param(const std::string& parameter_name)
{
    T value;
    if (this->get_parameter(parameter_name, value)) {
        return value;
    } else {
        RCLCPP_WARN(this->get_logger(), "Parameter [%s] not found, using default value.", parameter_name.c_str());
        return T();
    }
}