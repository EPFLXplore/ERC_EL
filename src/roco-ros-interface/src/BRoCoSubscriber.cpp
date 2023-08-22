/*
 * BRoCoSubscriber.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoSubscriber.h"
#include "BRoCoManager.h"
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
    RCLCPP_INFO(parent->get_logger(), "Creating subscribers");
    this->spectro_req_sub = parent->create_subscription<avionics_interfaces::msg::SpectroRequest>
        (get_prefix() + get_param<std::string>("SPECTRO_REQ_TOPIC"), 10, std::bind(&BRoCoSubscriber::spectroReqCallback, this, _1));
    this->servo_req_sub = parent->create_subscription<avionics_interfaces::msg::ServoRequest>
        (get_prefix() + get_param<std::string>("SERVO_REQ_TOPIC"), 10, std::bind(&BRoCoSubscriber::servoReqCallback, this, _1));
    this->laser_req_sub = parent->create_subscription<avionics_interfaces::msg::LaserRequest>
        (get_prefix() + get_param<std::string>("LASER_REQ_TOPIC"), 10, std::bind(&BRoCoSubscriber::laserReqCallback,this,  _1));
    this->led_req_sub = parent->create_subscription<avionics_interfaces::msg::LEDRequest>
        (get_prefix() + get_param<std::string>("LED_REQ_TOPIC"), 10, std::bind(&BRoCoSubscriber::ledReqCallback, this, _1));

    this->mass_config_req_sub = parent->create_subscription<avionics_interfaces::msg::MassConfigRequestJetson>
        (get_prefix() + get_param<std::string>("MASS_CONFIG_REQ_JETSON_TOPIC"), 10, std::bind(&BRoCoSubscriber::massConfigReqCallback, this, _1));
    RCLCPP_INFO(parent->get_logger(), "Subscribers created");
}

void BRoCoSubscriber::spectroReqCallback(const avionics_interfaces::msg::SpectroRequest::SharedPtr msg) {
    uint32_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_DRILL_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending spectro request to node ID " + std::to_string(id) + "...");
    static SpectroPacket packet;
    packet.measure = msg->measure;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}
void BRoCoSubscriber::servoReqCallback(const avionics_interfaces::msg::ServoRequest::SharedPtr msg) {
    uint32_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("HD_NODE_ID");
    RCLCPP_INFO(parent->get_logger(), "Sending servo request to node ID " + std::to_string(id) + "...");
    static ServoPacket packet;
    packet.channel = msg->channel;
    packet.angle = msg->angle;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::laserReqCallback(const avionics_interfaces::msg::LaserRequest::SharedPtr msg) {
    uint32_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("HD_NODE_ID");
    RCLCPP_INFO(parent->get_logger(), "Sending laser request to node ID " + std::to_string(id) + "...");
    static LaserPacket packet;
    packet.enable = msg->enable;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::ledReqCallback(const avionics_interfaces::msg::LEDRequest::SharedPtr msg) {
    uint32_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("NAV_NODE_ID");
    RCLCPP_INFO(parent->get_logger(), "Sending LED request to node ID " + std::to_string(id) + "...");
    static LEDPacket packet;
    packet.state = msg->state;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::massConfigReqCallback(const avionics_interfaces::msg::MassConfigRequestJetson::SharedPtr msg) {
    uint32_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("MASS_DRILL_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending Mass config to node ID " + std::to_string(id) + "...");
    static MassConfigPacket packet;
    packet.remote_command = msg->remote_command;
    packet.set_offset = msg->set_offset;
    packet.set_alpha = msg->set_alpha;
    packet.set_channels_status = msg->set_channels_status;

    for (uint8_t i = 0; i < 4; ++i) {
        packet.offset[i] = msg->offset[i];
        packet.scale[i] = msg->scale[i];
        if (msg->scale[i] = 0) {
            RCLCPP_INFO(parent->get_logger(), "Scale for channel %d is zero, not sending scale configuration", i+1);
            packet.set_scale = false;
        }
        packet.enabled_channels[i] = msg->enabled_channels[i];
    }

    packet.alpha = msg->alpha;

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

uint32_t BRoCoSubscriber::get_node_id(std::string node_name) {
    return dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
}

void BRoCoSubscriber::set_destination_id(std::string node_name) {
    uint32_t id = dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

void BRoCoSubscriber::set_destination_id(uint32_t id) {
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

std::string BRoCoSubscriber::get_prefix() {
  return dynamic_cast<BRoCoManager*>(parent)->get_prefix();
}

std::string BRoCoSubscriber::get_bus() {
  return dynamic_cast<BRoCoManager*>(parent)->get_bus();
}

template <typename T>
void BRoCoSubscriber::set_param_calib(const std::string& sensor, const std::string& parameter_name, const T& value) {
    dynamic_cast<BRoCoManager*>(parent)->set_param_calib(sensor, parameter_name, value);
}

template <typename T>
T BRoCoSubscriber::get_param(const std::string& parameter_name) {
  return dynamic_cast<BRoCoManager*>(parent)->get_param<T>(parameter_name);
}