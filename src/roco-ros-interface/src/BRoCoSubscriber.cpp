/*
 * BRoCoSubscriber.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoSubscriber.h"
#include "BRoCoManager.h"
#include "BRoCo/CanSocketDriver.h"

#include "Utils.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

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
    this->pot_config_req_sub = parent->create_subscription<avionics_interfaces::msg::PotConfigRequestJetson>
        (get_prefix() + get_param<std::string>("POT_CONFIG_REQ_JETSON_TOPIC"), 10, std::bind(&BRoCoSubscriber::potConfigReqCallback, this, _1));
    this->servo_config_req_sub = parent->create_subscription<avionics_interfaces::msg::ServoConfigRequestJetson>
        (get_prefix() + get_param<std::string>("SERVO_CONFIG_REQ_JETSON_TOPIC"), 10, std::bind(&BRoCoSubscriber::servoConfigReqCallback, this, _1));
    this->accel_config_req_sub = parent->create_subscription<avionics_interfaces::msg::AccelConfigRequestJetson>
        (get_prefix() + get_param<std::string>("ACCEL_CONFIG_REQ_JETSON_TOPIC"), 10, std::bind(&BRoCoSubscriber::accelConfigReqCallback, this, _1));
    this->gyro_config_req_sub = parent->create_subscription<avionics_interfaces::msg::GyroConfigRequestJetson>
        (get_prefix() + get_param<std::string>("GYRO_CONFIG_REQ_JETSON_TOPIC"), 10, std::bind(&BRoCoSubscriber::gyroConfigReqCallback, this, _1));
    this->mag_config_req_sub = parent->create_subscription<avionics_interfaces::msg::MagConfigRequestJetson>
        (get_prefix() + get_param<std::string>("MAG_CONFIG_REQ_JETSON_TOPIC"), 10, std::bind(&BRoCoSubscriber::magConfigReqCallback, this, _1));
    
    this->mass_drill_calib_offset_sub = parent->create_subscription<avionics_interfaces::msg::MassCalibOffset>
        (get_prefix() + get_param<std::string>("DRILL_MASS_CALIB_OFFSET_TOPIC"), 10, std::bind(&BRoCoSubscriber::massDrillCalibOffsetCallback, this, _1));
    this->mass_container_calib_offset_sub = parent->create_subscription<avionics_interfaces::msg::MassCalibOffset>
        (get_prefix() + get_param<std::string>("CONTAINER_MASS_CALIB_OFFSET_TOPIC"), 10, std::bind(&BRoCoSubscriber::massContainerCalibOffsetCallback, this, _1));
    this->mass_drill_calib_scale_sub = parent->create_subscription<avionics_interfaces::msg::MassCalibScale>
        (get_prefix() + get_param<std::string>("DRILL_MASS_CALIB_SCALE_TOPIC"), 10, std::bind(&BRoCoSubscriber::massDrillCalibScaleCallback, this, _1));
    this->mass_container_calib_scale_sub = parent->create_subscription<avionics_interfaces::msg::MassCalibScale>
        (get_prefix() + get_param<std::string>("CONTAINER_MASS_CALIB_SCALE_TOPIC"), 10, std::bind(&BRoCoSubscriber::massContainerCalibScaleCallback, this, _1));
this->imu_calib_sub = parent->create_subscription<avionics_interfaces::msg::ImuCalib>
        (get_prefix() + get_param<std::string>("IMU_CALIB_TOPIC"), 10, std::bind(&BRoCoSubscriber::imuCalibCallback, this, _1));


    RCLCPP_INFO(parent->get_logger(), "Subscribers created");
}

void BRoCoSubscriber::spectroReqCallback(const avionics_interfaces::msg::SpectroRequest::SharedPtr msg) {
    uint16_t id = 0;
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
    uint16_t id = 0;
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
    uint16_t id = 0;
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
    uint16_t id = 0;
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
    uint16_t id = 0;
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

void BRoCoSubscriber::potConfigReqCallback(const avionics_interfaces::msg::PotConfigRequestJetson::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("NAV_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending Potentiometer config to node ID " + std::to_string(id) + "...");
    static PotentiometerConfigPacket packet;
    packet.remote_command = msg->remote_command;
    packet.set_min_voltages = msg->set_min_voltages;
    packet.set_max_voltages = msg->set_max_voltages;
    packet.set_min_angles = msg->set_min_angles;
    packet.set_max_angles = msg->set_max_angles;
    packet.set_channels_status = msg->set_channels_status;

    float min_voltages[4];
    float max_voltages[4];
    float min_angles[4];
    float max_angles[4];

    for (uint8_t i = 0; i < 4; ++i) {
        min_voltages[i] = msg->min_voltages[i];
        max_voltages[i] = msg->max_voltages[i];
        min_angles[i] = msg->min_angles[i];
        max_angles[i] = msg->max_angles[i];
    }

    packet.min_voltages_max_val = get_max_val(min_voltages, 4);
    packet.max_voltages_max_val = get_max_val(max_voltages, 4);
    packet.min_angles_max_val = get_max_val(min_angles, 4);
    packet.max_angles_max_val = get_max_val(max_angles, 4);

    for (uint8_t i = 0; i < 4; ++i) {
        packet.min_voltages[i] = floatToScaledUInt16(min_voltages[i], packet.min_voltages_max_val);
        packet.max_voltages[i] = floatToScaledUInt16(max_voltages[i], packet.max_voltages_max_val);
        packet.min_angles[i] = floatToScaledUInt16(min_angles[i], packet.min_angles_max_val);
        packet.max_angles[i] = floatToScaledUInt16(max_angles[i], packet.max_angles_max_val);
        packet.enabled_channels[i] = msg->enabled_channels[i];
    }

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::servoConfigReqCallback(const avionics_interfaces::msg::ServoConfigRequestJetson::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("HD_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending Servo config to node ID " + std::to_string(id) + "...");
    static ServoConfigPacket packet;
    packet.remote_command = msg->remote_command;
    packet.set_min_duty = msg->set_min_duty;
    packet.set_max_duty = msg->set_max_duty;
    packet.set_min_angles = msg->set_min_angles;
    packet.set_max_angles = msg->set_max_angles;

    float min_duty[4];
    float max_duty[4];
    float min_angles[4];
    float max_angles[4];

    for (uint8_t i = 0; i < 4; ++i) {
        min_duty[i] = msg->min_duty[i];
        max_duty[i] = msg->max_duty[i];
        min_angles[i] = msg->min_angles[i];
        max_angles[i] = msg->max_angles[i];
    }

    packet.min_duty_max_val = get_max_val(min_duty, 4);
    packet.max_duty_max_val = get_max_val(max_duty, 4);
    packet.min_angles_max_val = get_max_val(min_angles, 4);
    packet.max_angles_max_val = get_max_val(max_angles, 4);

    for (uint8_t i = 0; i < 4; ++i) {
        packet.min_duty[i] = floatToScaledUInt16(min_duty[i], packet.min_duty_max_val);
        packet.max_duty[i] = floatToScaledUInt16(max_duty[i], packet.max_duty_max_val);
        packet.min_angles[i] = floatToScaledUInt16(min_angles[i], packet.min_angles_max_val);
        packet.max_angles[i] = floatToScaledUInt16(max_angles[i], packet.max_angles_max_val);
    }
    
    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::accelConfigReqCallback(const avionics_interfaces::msg::AccelConfigRequestJetson::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("NAV_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending Accel config to node ID " + std::to_string(id) + "...");
    static AccelConfigPacket packet;
    packet.remote_command = msg->remote_command;
    packet.set_bias = msg->set_bias;
    packet.set_transform = msg->set_transform;

    for (uint8_t i = 0; i < 3; ++i) 
        packet.bias[i] = msg->bias[i];

    for (uint8_t i = 0; i < 9; ++i)
        packet.transform[i] = msg->transform[i];

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    // set_destination_id(get_node_id("GENERAL_NODE_ID"));
    // set_destination_id(4);
    bus->send(&packet);
}

void BRoCoSubscriber::gyroConfigReqCallback(const avionics_interfaces::msg::GyroConfigRequestJetson::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("NAV_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending Gyro config to node ID " + std::to_string(id) + "...");
    static GyroConfigPacket packet;
    packet.remote_command = msg->remote_command;
    packet.set_bias = msg->set_bias;

    for (uint8_t i = 0; i < 3; ++i) 
        packet.bias[i] = msg->bias[i];

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    // set_destination_id(get_node_id("GENERAL_NODE_ID"));
    // set_destination_id(4);
    bus->send(&packet);
}

void BRoCoSubscriber::magConfigReqCallback(const avionics_interfaces::msg::MagConfigRequestJetson::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("NAV_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending Mag config to node ID " + std::to_string(id) + "...");
    static MagConfigPacket packet;
    packet.remote_command = msg->remote_command;
    packet.set_hard_iron = msg->set_hard_iron;
    packet.set_soft_iron = msg->set_soft_iron;

    for (uint8_t i = 0; i < 3; ++i) 
        packet.hard_iron[i] = msg->hard_iron[i];

    for (uint8_t i = 0; i < 9; ++i)
        packet.soft_iron[i] = msg->soft_iron[i];

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    // set_destination_id(get_node_id("GENERAL_NODE_ID"));
    // set_destination_id(4);
    bus->send(&packet);
}


void BRoCoSubscriber::massDrillCalibOffsetCallback(const avionics_interfaces::msg::MassCalibOffset::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_DRILL_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending offset calibration request to node ID " + std::to_string(id) + "...");
    static MassCalibPacket packet;
    packet.calib_offset = true;
    packet.calib_scale = false;
    packet.channel = msg->channel;
    packet.expected_weight = 1;

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::massContainerCalibOffsetCallback(const avionics_interfaces::msg::MassCalibOffset::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_CONTAINER_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending offset calibration request to node ID " + std::to_string(id) + "...");
    static MassCalibPacket packet;
    packet.calib_offset = true;
    packet.calib_scale = false;
    packet.channel = msg->channel;
    packet.expected_weight = 1;

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::massDrillCalibScaleCallback(const avionics_interfaces::msg::MassCalibScale::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_DRILL_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending scale calibration request to node ID " + std::to_string(id) + "...");
    static MassCalibPacket packet;
    packet.calib_offset = false;
    packet.calib_scale = true;
    packet.channel = msg->channel;
    packet.expected_weight = msg->expected_weight;

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::massContainerCalibScaleCallback(const avionics_interfaces::msg::MassCalibScale::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_CONTAINER_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending scale calibration request to node ID " + std::to_string(id) + "...");
    static MassCalibPacket packet;
    packet.calib_offset = false;
    packet.calib_scale = true;
    packet.channel = msg->channel;
    packet.expected_weight = msg->expected_weight;

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::imuCalibCallback(const avionics_interfaces::msg::ImuCalib::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("NAV_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending IMU calibration request to node ID " + std::to_string(id) + "...");
    static ImuCalibPacket packet;
    packet.calib_offset_accel = msg->calib_offset_accel;
    packet.calib_offset_gyro = msg->calib_offset_gyro;

    MAKE_IDENTIFIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

uint32_t BRoCoSubscriber::get_node_id(std::string node_name) {
    return dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
}

void BRoCoSubscriber::set_destination_id(std::string node_name) {
    uint16_t id = dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

void BRoCoSubscriber::set_destination_id(uint16_t id) {
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