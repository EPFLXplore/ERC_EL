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
#include "avionics_interfaces/srv/servo_request.hpp"

#define MAKE_IDENTIFIABLE(PACKET) (PACKET).id = 0x000;

class BRoCoSubscriber {
public:
    BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent);

    void performServoRequest();

private:
    void set_destination_id(uint32_t id);

    void timer_ping_callback();
    // void servo_request_callback();

    rclcpp::Clock::SharedPtr clk;
    rclcpp::TimerBase::SharedPtr timer;
    CANBus* bus;
    rclcpp::Node* parent;
};

#endif /* BROCO_SUBSCRIBER_H */