/*
 * ServiceManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "ServiceManager.h"


ServiceManager::ServiceManager() : Node("service_manager") {
    
    this->declare_parameter("JETSON_NODE_ID");
    this->declare_parameter("SC_CONTAINER_NODE_ID");
    this->declare_parameter("SC_DRILL_NODE_ID");
    this->declare_parameter("NAV_NODE_ID");
    this->declare_parameter("HD_NODE_ID");
    this->declare_parameter("GENERAL_NODE_ID");
    this->declare_parameter("MAX_NUMBER_NODES");

    this->declare_parameter("SPECTRO_REQ_TOPIC");
    this->declare_parameter("SERVO_REQ_TOPIC");
    this->declare_parameter("LASER_REQ_TOPIC");
    this->declare_parameter("LED_REQ_TOPIC");

    max_number_nodes = get_param<uint32_t>("MAX_NUMBER_NODES");
}

ServiceManager::~ServiceManager() {
    RCLCPP_INFO(this->get_logger(), "Deleting Service Manager");
}