/*
 * ConverterPublisher.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "ConverterPublisher.h"
#include "ConverterManager.h"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

ConverterPublisher::ConverterPublisher(rclcpp::Node* parent) : parent(parent) {
    RCLCPP_INFO(parent->get_logger(), "Creating converted publishers");

    this->imu_pub = parent->create_publisher<sensor_msgs::msg::Imu>(get_param<std::string>("IMU_NAV_TOPIC"), 10);
    this->mag_cal_pub = parent->create_publisher<sensor_msgs::msg::MagneticField>(get_param<std::string>("MAG_CAL_TOPIC"), 10);
    this->mag_raw_pub = parent->create_publisher<sensor_msgs::msg::MagneticField>(get_param<std::string>("MAG_RAW_TOPIC"), 10);

    this->imu_sub = parent->create_subscription<custom_msg::msg::Imu>
        (get_param<std::string>("IMU_TOPIC"), 10, std::bind(&ConverterPublisher::imuCallback, this, _1));
    this->mag_sub = parent->create_subscription<custom_msg::msg::Mag>
        (get_param<std::string>("MAG_TOPIC"), 10, std::bind(&ConverterPublisher::magCallback, this, _1));
}

void ConverterPublisher::imuCallback(const custom_msg::msg::Imu::SharedPtr msg) {
    imu_pub->publish(msg->imu);
}

void ConverterPublisher::magCallback(const custom_msg::msg::Mag::SharedPtr msg) {
    mag_cal_pub->publish(msg->mag_cal);
    mag_raw_pub->publish(msg->mag_raw);
}


template <typename T>
T ConverterPublisher::get_param(const std::string& parameter_name) {
    return dynamic_cast<ConverterManager*>(parent)->get_param<T>(parameter_name);
}