/*
 * ConverterPublisher.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef CONVERTER_PUBLISHER_H
#define CONVERTER_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"

#include "avionics_interfaces/msg/imu.hpp"
#include "avionics_interfaces/msg/mag.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

class ConverterPublisher {
public:
    ConverterPublisher(rclcpp::Node* parent);

private:
    rclcpp::Node* parent;

    template <typename T>
    T get_param(const std::string& parameter_name);

    rclcpp::Subscription<avionics_interfaces::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<avionics_interfaces::msg::Mag>::SharedPtr mag_sub;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_cal_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_raw_pub;

    void imuCallback(const avionics_interfaces::msg::Imu::SharedPtr msg);
    void magCallback(const avionics_interfaces::msg::Mag::SharedPtr msg);
};

#endif /* CONVERTER_PUBLISHER_H */