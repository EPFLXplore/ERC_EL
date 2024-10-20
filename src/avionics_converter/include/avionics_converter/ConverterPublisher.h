/*
 * ConverterPublisher.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef CONVERTER_PUBLISHER_H
#define CONVERTER_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"

#include "custom_msg/msg/imu.hpp"
#include "custom_msg/msg/mag.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

class ConverterPublisher {
public:
    ConverterPublisher(rclcpp::Node* parent);

private:
    rclcpp::Node* parent;

    template <typename T>
    T get_param(const std::string& parameter_name);

    rclcpp::Subscription<custom_msg::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<custom_msg::msg::Mag>::SharedPtr mag_sub;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_cal_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_raw_pub;

    void imuCallback(const custom_msg::msg::Imu::SharedPtr msg);
    void magCallback(const custom_msg::msg::Mag::SharedPtr msg);
};

#endif /* CONVERTER_PUBLISHER_H */