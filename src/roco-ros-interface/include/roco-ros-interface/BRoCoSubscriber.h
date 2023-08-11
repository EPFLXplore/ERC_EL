#ifndef BROCO_SUBSCRIBER_H
#define BROCO_SUBSCRIBER_H

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "BRoCo/CanSocketDriver.h"
#include "BRoCo/CANBus.h"
#include "Protocol/Protocol.h"

using namespace std::chrono_literals;

class BRoCoSubscriber {
public:
    BRoCoSubscriber(std::shared_ptr<CANBus> bus, CanSocketDriver* driver, rclcpp::Node* parent);

private:
    std::shared_ptr<CANBus> bus;
    CanSocketDriver* driver;
    rclcpp::Clock::SharedPtr clk;
    rclcpp::TimerBase::SharedPtr timer;

    void callback();
};

#endif /* BROCO_SUBSCRIBER_H */