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

#define MAKE_IDENTIFIABLE(PACKET) (PACKET).id = 0x000;

class BRoCoSubscriber {
public:
    BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent);

    void performServoRequest();

private:
    void set_destination_id(uint32_t id);
    void set_destination_id(std::string node_name);

    std::string get_ns();
    std::string get_bus();

    template <typename T>
    void set_param(const std::string& parameter_name, const T& value);
    
    rclcpp::Subscription<avionics_interfaces::msg::SpectroRequest>::SharedPtr spectro_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::ServoRequest>::SharedPtr servo_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::LaserRequest>::SharedPtr laser_req_sub;
    rclcpp::Subscription<avionics_interfaces::msg::LEDRequest>::SharedPtr led_req_sub;

    void spectroReqCallback(const avionics_interfaces::msg::SpectroRequest::SharedPtr msg);
    void servoReqCallback(const avionics_interfaces::msg::ServoRequest::SharedPtr msg);
    void laserReqCallback(const avionics_interfaces::msg::LaserRequest::SharedPtr msg);
    void ledReqCallback(const avionics_interfaces::msg::LEDRequest::SharedPtr msg);
    

    rclcpp::Clock::SharedPtr clk;
    rclcpp::TimerBase::SharedPtr timer;
    CANBus* bus;
    rclcpp::Node* parent;
};

#endif /* BROCO_SUBSCRIBER_H */