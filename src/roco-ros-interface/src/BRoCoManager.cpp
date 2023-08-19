/*
 * BRoCoManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "BRoCoManager.h"


BRoCoManager::BRoCoManager() : Node("broco_manager") {

  // Declare a parameter to hold the bus name
  this->declare_parameter("bus");

  if (!this->get_parameter("bus", bus_name)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get the 'bus' parameter");
  }
  RCLCPP_INFO(this->get_logger(), "Selected CAN bus: %s", bus_name.c_str());

  this->declare_parameter("namespace");
  if (!this->get_parameter("namespace", ns)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get the 'namespace' parameter");
  }
  ns = "/" + ns;
  RCLCPP_INFO(this->get_logger(), "Namespace: %s", ns.c_str());

  // Load parameters from the specified YAML file
  this->declare_parameter("JETSON_NODE_ID");
  this->declare_parameter("SC_CONTAINER_NODE_ID");
  this->declare_parameter("SC_DRILL_NODE_ID");
  this->declare_parameter("NAV_NODE_ID");
  this->declare_parameter("HD_NODE_ID");
  this->declare_parameter("GENERAL_NODE_ID");
  this->declare_parameter("MAX_NUMBER_NODES");
  this->declare_parameter("NODE_PING_INTERVAL");
  this->declare_parameter("NODE_STATE_WATCHDOG_TIMEOUT");
  this->declare_parameter("NODE_STATE_PUBLISH_INTERVAL");

  this->declare_parameter("CONNECTION_RETRY_NUM");
  this->declare_parameter("CONNECTION_RETRY_INTERVAL");

  
  // Create a timer to periodically attempt connection
  // retry_timer = this->create_wall_timer(
  //     std::chrono::milliseconds(get_param<uint32_t>("CONNECTION_RETRY_INTERVAL")),
  //     std::bind(&BRoCoManager::retryConnection, this)
  // );

  // RCLCPP_INFO(this->get_logger(), "JETSON_NODE_ID: " + std::to_string(get_param<uint32_t>("JETSON_NODE_ID")));
  // RCLCPP_INFO(this->get_logger(), "SC_CONTAINER_NODE_ID: " + std::to_string(get_param<uint32_t>("SC_CONTAINER_NODE_ID")));
  // RCLCPP_INFO(this->get_logger(), "SC_DRILL_NODE_ID: " + std::to_string(get_param<uint32_t>("SC_DRILL_NODE_ID")));
  // RCLCPP_INFO(this->get_logger(), "NAV_NODE_ID: " + std::to_string(get_param<uint32_t>("NAV_NODE_ID")));
  // RCLCPP_INFO(this->get_logger(), "HD_NODE_ID: " + std::to_string(get_param<uint32_t>("HD_NODE_ID")));
  // RCLCPP_INFO(this->get_logger(), "GENERAL_NODE_ID: " + std::to_string(get_param<uint32_t>("GENERAL_NODE_ID")));
  // RCLCPP_INFO(this->get_logger(), "MAX_NUMBER_NODES: " + std::to_string(get_param<uint32_t>("MAX_NUMBER_NODES")));
  // RCLCPP_INFO(this->get_logger(), "NODE_PING_INTERVAL: " + std::to_string(get_param<uint32_t>("NODE_PING_INTERVAL")));
  // RCLCPP_INFO(this->get_logger(), "NODE_STATE_WATCHDOG_TIMEOUT: " + std::to_string(get_param<uint32_t>("NODE_STATE_WATCHDOG_TIMEOUT")));
  // RCLCPP_INFO(this->get_logger(), "NODE_STATE_PUBLISH_INTERVAL: " + std::to_string(get_param<uint32_t>("NODE_STATE_PUBLISH_INTERVAL")));
  // RCLCPP_INFO(this->get_logger(), "CONNECTION_RETRY_NUM: " + std::to_string(get_param<uint32_t>("CONNECTION_RETRY_NUM")));
  // RCLCPP_INFO(this->get_logger(), "CONNECTION_RETRY_INTERVAL: " + std::to_string(get_param<uint32_t>("CONNECTION_RETRY_INTERVAL")));

    // Check if maximum retry attempts have been reached
  if (retry_count >= get_param<uint32_t>("CONNECTION_RETRY_NUM")) {
      RCLCPP_ERROR(this->get_logger(), "Maximum retry attempts reached. Giving up.");
      retry_timer->cancel();  // Stop the retry attempts
      rclcpp::shutdown();
      return;
  }

  // Create CanSocketDriver instance
  this->driver = new CanSocketDriver(bus_name.c_str());


      RCLCPP_INFO(this->get_logger(), "CAN driver connected.");
      createPubSub();
}

BRoCoManager::~BRoCoManager() {
  RCLCPP_INFO(this->get_logger(), "Deleting sub and pub");
  delete this->sub;
  delete this->pub;
  delete this->bus;
  delete this->driver;
}

// void BRoCoManager::retryConnection() {

//   // Check if maximum retry attempts have been reached
//   if (retry_count >= get_param<uint32_t>("CONNECTION_RETRY_NUM")) {
//       RCLCPP_ERROR(this->get_logger(), "Maximum retry attempts reached. Giving up.");
//       retry_timer->cancel();  // Stop the retry attempts
//       rclcpp::shutdown();
//       return;
//   }

//   // Create CanSocketDriver instance
//   this->driver = new CanSocketDriver(bus_name.c_str());

//   // Check if the driver is connected
//   // if (driver->isConnected()) {
//       RCLCPP_INFO(this->get_logger(), "CAN driver connected.");
//       retry_timer->cancel();  // Stop the retry attempts
//       createPubSub();
//   // } else {
//   //     ++retry_count;
//   //     RCLCPP_WARN(this->get_logger(), "CAN driver not connected, retrying... (Attempt %d/%d)", retry_count, get_param<uint32_t>("CONNECTION_RETRY_NUM"));
//   //     delete driver;  // Clean up the previous instance
//   //     driver = nullptr;
//   // }
// }

std::string BRoCoManager::get_ns() {
  return ns;
}

std::string BRoCoManager::get_bus() {
  return bus_name;
}

void BRoCoManager::createPubSub() {
  this->bus = new CANBus(this->driver);
  this->pub = new BRoCoPublisher(this->bus, this);
  this->sub = new BRoCoSubscriber(this->bus, this);
  this->driver->start_reception();
}