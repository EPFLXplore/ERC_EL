#ifndef BROCO_SUBSCRIBER_H
#define BROCO_SUBSCRIBER_H

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "BRoCo/CANBus.h"
#include "Protocol/Protocol.h"

using namespace std::chrono_literals;

class BRoCoSubscriber {
public:
    BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent);

private:
    CANBus* bus;
    rclcpp::Clock::SharedPtr clk;
    rclcpp::TimerBase::SharedPtr timer;
    void set_destination_id(uint32_t id);

    void callback();
};

#endif /* BROCO_SUBSCRIBER_H */