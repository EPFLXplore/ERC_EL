/*
 * BRoCoSubscriber.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoSubscriber.h"
#include "BRoCo/CanSocketDriver.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;
uint32_t seq = 0;

BRoCoSubscriber::BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent) : bus(bus), parent(parent) {
    this->clk = parent->get_clock();
    this->timer = parent->create_wall_timer(1000ms, std::bind(&BRoCoSubscriber::timerPingCallback, this));
    this->spectro_req_sub = parent->create_subscription<avionics_interfaces::msg::SpectroRequest>
        ("/spectro_req", 10, std::bind(&BRoCoSubscriber::spectroReqCallback, this, _1));
    this->servo_req_sub = parent->create_subscription<avionics_interfaces::msg::ServoRequest>
        ("/servo_req", 10, std::bind(&BRoCoSubscriber::servoReqCallback, this, _1));
    this->laser_req_sub = parent->create_subscription<avionics_interfaces::msg::LaserRequest>
        ("/spectro_req", 10, std::bind(&BRoCoSubscriber::laserReqCallback,this,  _1));
    this->led_req_sub = parent->create_subscription<avionics_interfaces::msg::LEDRequest>
        ("/spectro_req", 10, std::bind(&BRoCoSubscriber::ledReqCallback, this, _1));
}

void BRoCoSubscriber::timerPingCallback() {
    static PingPacket packet;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(0x7FF);
    bus->send(&packet);
}

void BRoCoSubscriber::spectroReqCallback(const avionics_interfaces::msg::SpectroRequest::SharedPtr msg) {
    static SpectroPacket packet;
    packet.measure = msg->measure;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(0x7FF);
    bus->send(&packet);
}
void BRoCoSubscriber::servoReqCallback(const avionics_interfaces::msg::ServoRequest::SharedPtr msg) {
    static ServoPacket packet;
    packet.channel = msg->channel;
    packet.angle = msg->angle;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(0x7FF);
    bus->send(&packet);
}

void BRoCoSubscriber::laserReqCallback(const avionics_interfaces::msg::LaserRequest::SharedPtr msg) {
    static LaserPacket packet;
    packet.enable = msg->enable;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(0x7FF);
    bus->send(&packet);
}

void BRoCoSubscriber::ledReqCallback(const avionics_interfaces::msg::LEDRequest::SharedPtr msg) {
    static LEDPacket packet;
    packet.state = msg->state;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(0x7FF);
    bus->send(&packet);
}

void BRoCoSubscriber::set_destination_id(uint32_t id) {
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}