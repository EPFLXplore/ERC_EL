/*
 * ConverterManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "ConverterManager.h"


ConverterManager::ConverterManager() : Node("converter_manager") {

    // Topic names =================================
    // Publishers
    this->declare_parameter("IMU_TOPIC");
    this->declare_parameter("IMU_NAV_TOPIC");

    this->declare_parameter("MAG_TOPIC");
    this->declare_parameter("MAG_CAL_TOPIC");
    this->declare_parameter("MAG_RAW_TOPIC");

    this->pub = new ConverterPublisher(this);
}

ConverterManager::~ConverterManager() {
    RCLCPP_INFO(this->get_logger(), "Deleting Converter Manager");
    delete this->pub;
}