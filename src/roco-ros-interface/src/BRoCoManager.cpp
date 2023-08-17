/*
 * BRoCoManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "BRoCoManager.h"


BRoCoManager::BRoCoManager() : Node("broco_manager") {

  // Load parameters from the specified YAML file
  this->declare_parameter("JETSON_NODE_ID");
  this->declare_parameter("SC_CONTAINER_NODE_ID");
  this->declare_parameter("SC_DRILL_NODE_ID");
  this->declare_parameter("NAV_NODE_ID");
  this->declare_parameter("HD_NODE_ID");
  this->declare_parameter("GENERAL_NODE_ID");
  this->declare_parameter("MAX_NUMBER_NODES");
  this->declare_parameter("NODE_PING_PERIOD");
  this->declare_parameter("NODE_STATE_WATCHDOG_TIMEOUT");
  this->declare_parameter("NODE_STATE_PUBLISH_PERIOD");

  RCLCPP_DEBUG(this->get_logger(), "Creating publishers and subscribers...");
  this->driver = new CanSocketDriver("can0");
  // this->bus = new CANBus(this->driver);
  this->bus = new CANBus(this->driver);
  this->pub = new BRoCoPublisher(this->bus, this);
  this->sub = new BRoCoSubscriber(this->bus, this);

  RCLCPP_INFO(this->get_logger(), "JETSON_NODE_ID: " + std::to_string(get_param<uint32_t>("JETSON_NODE_ID")));
  RCLCPP_INFO(this->get_logger(), "SC_CONTAINER_NODE_ID: " + std::to_string(get_param<uint32_t>("SC_CONTAINER_NODE_ID")));
  RCLCPP_INFO(this->get_logger(), "SC_DRILL_NODE_ID: " + std::to_string(get_param<uint32_t>("SC_DRILL_NODE_ID")));
  RCLCPP_INFO(this->get_logger(), "NAV_NODE_ID: " + std::to_string(get_param<uint32_t>("NAV_NODE_ID")));
  RCLCPP_INFO(this->get_logger(), "HD_NODE_ID: " + std::to_string(get_param<uint32_t>("HD_NODE_ID")));
  RCLCPP_INFO(this->get_logger(), "GENERAL_NODE_ID: " + std::to_string(get_param<uint32_t>("GENERAL_NODE_ID")));
  RCLCPP_INFO(this->get_logger(), "MAX_NUMBER_NODES: " + std::to_string(get_param<uint32_t>("MAX_NUMBER_NODES")));
  RCLCPP_INFO(this->get_logger(), "NODE_PING_PERIOD: " + std::to_string(get_param<uint32_t>("NODE_PING_PERIOD")));
  RCLCPP_INFO(this->get_logger(), "NODE_STATE_WATCHDOG_TIMEOUT: " + std::to_string(get_param<uint32_t>("NODE_STATE_WATCHDOG_TIMEOUT")));
  RCLCPP_INFO(this->get_logger(), "NODE_STATE_PUBLISH_PERIOD: " + std::to_string(get_param<uint32_t>("NODE_STATE_PUBLISH_PERIOD")));
}

BRoCoManager::~BRoCoManager() {
    delete this->sub;
    delete this->pub;
    delete this->bus;
    delete this->driver;
}