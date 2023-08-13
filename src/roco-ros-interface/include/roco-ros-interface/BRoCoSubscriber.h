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
#include "avionics_interfaces/srv/servo_request.hpp"

class BRoCoSubscriber {
public:
    BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent);

    void performServoRequest();

private:
    void callback();
    void set_destination_id(uint32_t id);
    void handleServoResponse(uint8_t senderID, ServoResponsePacket* packet);
    rclcpp::Clock::SharedPtr clk;
    rclcpp::TimerBase::SharedPtr timer;
    CANBus* bus;
    rclcpp::Node* parent;
    // rclcpp::Service<avionics_interfaces::srv::ServoRequest>::SharedPtr servo_request_server_;

    // void handle_servo_request(
    //     const std::shared_ptr<avionics_interfaces::srv::ServoRequest::Request> request,
    //     std::shared_ptr<avionics_interfaces::srv::ServoRequest::Response> response);

    
    // std::promise<void>* response_promise_;
    // std::future<void> response_future_;
    // bool packet_success_;
};

#endif /* BROCO_SUBSCRIBER_H */