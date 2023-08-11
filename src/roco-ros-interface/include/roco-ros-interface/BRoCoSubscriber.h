#ifndef BROCO_SUBSCRIBER_H
#define BROCO_SUBSCRIBER_H

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "BRoCo/CanSocketDriver.h"
#include "BRoCo/IOBus.h"
#include "Protocol/Protocol.h"

using namespace std::chrono_literals;

class BRoCoSubscriber {
public:
    BRoCoSubscriber(IOBus* bus, CanSocketDriver* driver, rclcpp::Node* parent);

private:
    IOBus* bus;
    CanSocketDriver* driver;
    rclcpp::Clock::SharedPtr clk;
    rclcpp::TimerBase::SharedPtr timer;

    void callback();
};

#endif /* BROCO_SUBSCRIBER_H */