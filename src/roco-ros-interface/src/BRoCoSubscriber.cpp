/*
 * BRoCoSubscriber.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoSubscriber.h"
#include "BRoCo/CanSocketDriver.h"
#include "Time.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
uint32_t seq = 0;

BRoCoSubscriber::BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent) : bus(bus), parent(parent) {
    this->clk = parent->get_clock();
    this->timer = parent->create_wall_timer(1000ms, std::bind(&BRoCoSubscriber::timer_ping_callback, this));

}


void BRoCoSubscriber::timer_ping_callback() {
    static PingPacket packet;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(0x7FF);
    bus->send(&packet);
}

void BRoCoSubscriber::set_destination_id(uint32_t id) {
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}