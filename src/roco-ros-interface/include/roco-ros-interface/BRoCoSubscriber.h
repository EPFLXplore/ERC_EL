/*
 * BRoCoSubscriber.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef BROCO_SUBSCRIBER_H
#define BROCO_SUBSCRIBER_H

#include <chrono>
#include <memory>
#include <functional>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "BRoCo/CANBus.h"
#include "Protocol/Protocol.h"

#include "avionics_interfaces/msg/spectro_request.hpp"
#include "avionics_interfaces/msg/servo_request.hpp"
#include "avionics_interfaces/msg/laser_request.hpp"
#include "avionics_interfaces/msg/led_request.hpp"

#include "avionics_interfaces/msg/mass_config_request_jetson.hpp"
#include "avionics_interfaces/msg/pot_config_request_jetson.hpp"
#include "avionics_interfaces/msg/servo_config_request_jetson.hpp"
#include "avionics_interfaces/msg/accel_config_request_jetson.hpp"
#include "avionics_interfaces/msg/gyro_config_request_jetson.hpp"
#include "avionics_interfaces/msg/mag_config_request_jetson.hpp"

#define MAKE_IDENTIFIABLE(PACKET) (PACKET).id = 0x000;

class BRoCoSubscriber {
public:
    BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent);

    void performServoRequest();

private:
    uint32_t get_node_id(std::string node_name);
    void set_destination_id(uint32_t id);
    void set_destination_id(std::string node_name);

    std::string get_prefix();
    std::string get_bus();

    template <typename T>
    void set_param_calib(const std::string& sensor, const std::string& parameter_name, const T& value);

    template <typename T>
    T get_param(const std::string& parameter_name);
    
    rclcpp::Subscription<avionics_interfaces::msg::SpectroRequest>::SharedPtr spectro_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::ServoRequest>::SharedPtr servo_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::LaserRequest>::SharedPtr laser_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::LEDRequest>::SharedPtr led_req_sub;

    rclcpp::Subscription<avionics_interfaces::msg::MassConfigRequestJetson>::SharedPtr mass_config_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::PotConfigRequestJetson>::SharedPtr pot_config_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::ServoConfigRequestJetson>::SharedPtr servo_config_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::AccelConfigRequestJetson>::SharedPtr accel_config_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::GyroConfigRequestJetson>::SharedPtr gyro_config_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::MagConfigRequestJetson>::SharedPtr mag_config_req_sub;

    void spectroReqCallback(const avionics_interfaces::msg::SpectroRequest::SharedPtr msg);
    void servoReqCallback(const avionics_interfaces::msg::ServoRequest::SharedPtr msg);
    void laserReqCallback(const avionics_interfaces::msg::LaserRequest::SharedPtr msg);
    void ledReqCallback(const avionics_interfaces::msg::LEDRequest::SharedPtr msg);
    
    void massConfigReqCallback(const avionics_interfaces::msg::MassConfigRequestJetson::SharedPtr msg);
    void potConfigReqCallback(const avionics_interfaces::msg::PotConfigRequestJetson::SharedPtr msg);
    void servoConfigReqCallback(const avionics_interfaces::msg::ServoConfigRequestJetson::SharedPtr msg);
    void accelConfigReqCallback(const avionics_interfaces::msg::AccelConfigRequestJetson::SharedPtr msg);
    void gyroConfigReqCallback(const avionics_interfaces::msg::GyroConfigRequestJetson::SharedPtr msg);
    void magConfigReqCallback(const avionics_interfaces::msg::MagConfigRequestJetson::SharedPtr msg);


    rclcpp::Clock::SharedPtr clk;
    rclcpp::TimerBase::SharedPtr timer;
    CANBus* bus;
    rclcpp::Node* parent;
};

#endif /* BROCO_SUBSCRIBER_H */