/*
 * BRoCoPublisher.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoPublisher.h"
#include "BRoCoManager.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <functional>

using namespace std::chrono_literals;

BRoCoPublisher::BRoCoPublisher(CANBus* bus, rclcpp::Node* parent) : bus(bus), parent(parent) {

    this->clk = parent->get_clock();
    RCLCPP_INFO(parent->get_logger(), "Creating publishers");
    this->timer = parent->create_wall_timer(std::chrono::milliseconds(get_param<uint32_t>("NODE_PING_INTERVAL")), std::bind(&BRoCoPublisher::timerPingCallback, this));
    this->node_state_pub_timer = parent->create_wall_timer(std::chrono::milliseconds(get_param<uint32_t>("NODE_STATE_PUBLISH_INTERVAL")), std::bind(&BRoCoPublisher::nodeStateCallback, this));
    this->four_in_one_pub = parent->create_publisher<avionics_interfaces::msg::FourInOne>(get_prefix() + get_param<std::string>("FOUR_IN_ONE_TOPIC"), 10);
    this->npk_pub = parent->create_publisher<avionics_interfaces::msg::NPK>(get_prefix() + get_param<std::string>("NPK_TOPIC"), 10);
    this->voltage_pub = parent->create_publisher<avionics_interfaces::msg::Voltage>(get_prefix() + get_param<std::string>("VOLTAGE_TOPIC"), 10);
    this->drill_mass_pub = parent->create_publisher<avionics_interfaces::msg::MassArray>(get_prefix() + get_param<std::string>("DRILL_MASS_TOPIC"), 10);
    this->container_mass_pub = parent->create_publisher<avionics_interfaces::msg::MassArray>(get_prefix() + get_param<std::string>("CONTAINER_MASS_TOPIC"), 10);
    this->imu_pub = parent->create_publisher<avionics_interfaces::msg::Imu>(get_prefix() + get_param<std::string>("IMU_TOPIC"), 10);
    this->mag_pub = parent->create_publisher<avionics_interfaces::msg::Mag>(get_prefix() + get_param<std::string>("MAG_TOPIC"), 10);
    this->potentiometer_pub = parent->create_publisher<avionics_interfaces::msg::AngleArray>(get_prefix() + get_param<std::string>("POTENTIOMETER_TOPIC"), 10);
    this->spectro_response_pub = parent->create_publisher<avionics_interfaces::msg::SpectroResponse>(get_prefix() + get_param<std::string>("SPECTRO_TOPIC"), 10);
    this->laser_response_pub = parent->create_publisher<avionics_interfaces::msg::LaserResponse>(get_prefix() + get_param<std::string>("LASER_TOPIC"), 10);
    this->servo_response_pub = parent->create_publisher<avionics_interfaces::msg::ServoResponse>(get_prefix() + get_param<std::string>("SERVO_TOPIC"), 10);
    this->led_response_pub = parent->create_publisher<avionics_interfaces::msg::LEDResponse>(get_prefix() + get_param<std::string>("LED_TOPIC"), 10);
    this->node_state_pub = parent->create_publisher<avionics_interfaces::msg::NodeStateArray>(get_prefix() + get_param<std::string>("NODE_STATE_TOPIC"), 10);

    this->mass_config_req_pub = parent->create_publisher<avionics_interfaces::msg::MassConfigRequestMCU>(get_prefix() + get_param<std::string>("MASS_CONFIG_REQ_MCU_TOPIC"), 10);
    this->mass_config_response_pub = parent->create_publisher<avionics_interfaces::msg::MassConfigResponse>(get_prefix() + get_param<std::string>("MASS_CONFIG_TOPIC"), 10);
    RCLCPP_INFO(parent->get_logger(), "Publishers created");

    RCLCPP_INFO(parent->get_logger(), "Adding handles...");
    bus->handle<FOURINONEPacket>(std::bind(&BRoCoPublisher::handleFourInOnePacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<NPKPacket>(std::bind(&BRoCoPublisher::handleNPKPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<VoltmeterPacket>(std::bind(&BRoCoPublisher::handleVoltmeterPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<MassPacket>(std::bind(&BRoCoPublisher::handleMassPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<IMUPacket>(std::bind(&BRoCoPublisher::handleIMUPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<MagPacket>(std::bind(&BRoCoPublisher::handleMagPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<PotentiometerPacket>(std::bind(&BRoCoPublisher::handlePotentiometerPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<SpectroResponsePacket>(std::bind(&BRoCoPublisher::handleSpectroPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<LaserResponsePacket>(std::bind(&BRoCoPublisher::handleLaserPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<ServoResponsePacket>(std::bind(&BRoCoPublisher::handleServoPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<LEDResponsePacket>(std::bind(&BRoCoPublisher::handleLEDPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<PingPacket>(std::bind(&BRoCoPublisher::handlePingPacket, this, std::placeholders::_1, std::placeholders::_2));
    
    bus->handle<MassConfigRequestPacket>(std::bind(&BRoCoPublisher::handleMassConfigReqPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<MassConfigResponsePacket>(std::bind(&BRoCoPublisher::handleMassConfigPacket, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(parent->get_logger(), "Handles created");

    node_state.resize(get_param<uint32_t>("MAX_NUMBER_NODES"), false);
    watchdog_timers.resize(node_state.size());
    
    for (size_t i = 0; i < watchdog_timers.size(); ++i) {
        watchdog_timers[i] = parent->create_wall_timer(
            std::chrono::milliseconds(get_param<uint32_t>("NODE_STATE_WATCHDOG_TIMEOUT")),
            [this, i]() {
                this->watchdogCallback(i);
            }
        );
    }
}

void BRoCoPublisher::timerPingCallback() {
    static PingPacket packet;
    MAKE_IDENTIFIABLE(packet);
    set_destination_id("GENERAL_NODE_ID");
    bus->send(&packet);
}

void BRoCoPublisher::nodeStateCallback() {
    auto msg = avionics_interfaces::msg::NodeStateArray();
    for (int i = 0; i < node_state.size(); ++i)
    msg.node_state.push_back(node_state[i]);
    node_state_pub->publish(msg);
}

void BRoCoPublisher::watchdogCallback(size_t nodeID) {
    // This callback is triggered when the timer for nodeID expires
    node_state[nodeID] = false;
}

void BRoCoPublisher::handlePingPacket(uint8_t senderID, PingPacket* packet) {
    uint32_t id = packet->id;
    if (id < watchdog_timers.size()) {
        watchdog_timers[id]->reset();
    }
    // Update the node state for senderID to true
    if (id < node_state.size())
        node_state[id] = true;
}

void BRoCoPublisher::handleFourInOnePacket(uint8_t senderID, FOURINONEPacket* packet) {
    auto msg = avionics_interfaces::msg::FourInOne();

    msg.id = packet->id;

    msg.temperature = packet->temperature;
    msg.moisture = packet->moisture;
    msg.conductivity = packet->conductivity;
    msg.ph = packet->pH;

    four_in_one_pub->publish(msg);
}

void BRoCoPublisher::handleNPKPacket(uint8_t senderID, NPKPacket* packet) {
    auto msg = avionics_interfaces::msg::NPK();

    msg.id = packet->id;

    msg.nitrogen = packet->nitrogen;
    msg.phosphorus = packet->phosphorus;
    msg.potassium = packet->potassium;

    npk_pub->publish(msg);
}

void BRoCoPublisher::handleVoltmeterPacket(uint8_t senderID, VoltmeterPacket* packet) {
    auto msg = avionics_interfaces::msg::Voltage();

    msg.id = packet->id;

    msg.voltage = packet->voltage;

    voltage_pub->publish(msg);
}

void BRoCoPublisher::handleMassPacket(uint8_t senderID, MassPacket* packet) {
    auto msg = avionics_interfaces::msg::MassArray();

    msg.id = packet->id;

    for (uint8_t i = 0; i < msg.mass.size(); ++i)
        msg.mass[i] = packet->mass[i];
    if (packet->id == get_node_id("SC_DRILL_NODE_ID"))
        drill_mass_pub->publish(msg);
    else if (packet->id == get_node_id("SC_CONTAINER_NODE_ID"))
        container_mass_pub->publish(msg);
    else
        RCLCPP_INFO(parent->get_logger(), "Mass packet received but ID is not valid");
}

void BRoCoPublisher::handleIMUPacket(uint8_t senderID, IMUPacket* packet) {
    auto msg = avionics_interfaces::msg::Imu();

    msg.id = packet->id;

    msg.imu.header.stamp = clk->now();
    msg.imu.header.frame_id = "imu";

    msg.imu.angular_velocity.x = packet->angular[0];
    msg.imu.angular_velocity.y = packet->angular[1];
    msg.imu.angular_velocity.z = packet->angular[2];
    msg.imu.linear_acceleration.x = packet->acceleration[0];
    msg.imu.linear_acceleration.y = packet->acceleration[1];
    msg.imu.linear_acceleration.z = packet->acceleration[2];

    msg.imu.orientation.w = packet->orientation[0];
    msg.imu.orientation.x = packet->orientation[1];
    msg.imu.orientation.y = packet->orientation[2];
    msg.imu.orientation.z = packet->orientation[3];

    // To change

    msg.imu.orientation_covariance[0] = 1e-4;
    msg.imu.orientation_covariance[3] = 1e-4;
    msg.imu.orientation_covariance[6] = 1e-4;

    msg.imu.linear_acceleration_covariance[0] = 1.4e-3;
    msg.imu.linear_acceleration_covariance[1] = 1.0e-4;
    msg.imu.linear_acceleration_covariance[2] = 4.5e-5;
    msg.imu.linear_acceleration_covariance[3] = 1.0e-4;
    msg.imu.linear_acceleration_covariance[4] = 2.1e-3;
    msg.imu.linear_acceleration_covariance[5] = 2.5e-4;
    msg.imu.linear_acceleration_covariance[6] = 4.5e-5;
    msg.imu.linear_acceleration_covariance[7] = 2.5e-4;
    msg.imu.linear_acceleration_covariance[8] = 2.0e-3;

    msg.imu.angular_velocity_covariance[0] = 1.6e-2;
    msg.imu.angular_velocity_covariance[1] = -4.1e-5;
    msg.imu.angular_velocity_covariance[2] = 4.7e-3;
    msg.imu.angular_velocity_covariance[3] = -4.1e-5;
    msg.imu.angular_velocity_covariance[4] = 1.7e-2;
    msg.imu.angular_velocity_covariance[5] = 1.1e-4;
    msg.imu.angular_velocity_covariance[6] = 4.7e-3;
    msg.imu.angular_velocity_covariance[7] = 1.1e-4;
    msg.imu.angular_velocity_covariance[8] = 2.5e-2;

    imu_pub->publish(msg);
}

void BRoCoPublisher::handleMagPacket(uint8_t senderID, MagPacket* packet) {
    auto msg = avionics_interfaces::msg::Mag();

    msg.id = packet->id;

    msg.mag_raw.header.stamp = clk->now();
    msg.mag_cal.header.stamp = clk->now();
    msg.mag_raw.header.frame_id = "imu";
    msg.mag_cal.header.frame_id = "imu";

    msg.mag_raw.magnetic_field.x = packet->mag_raw[0]*1000000;
    msg.mag_raw.magnetic_field.y = packet->mag_raw[1]*1000000;
    msg.mag_raw.magnetic_field.z = packet->mag_raw[2]*1000000;

    msg.mag_cal.magnetic_field.x = packet->mag[0]*1000000;
    msg.mag_cal.magnetic_field.y = packet->mag[1]*1000000;
    msg.mag_cal.magnetic_field.z = packet->mag[2]*1000000;

    // To change
    msg.mag_raw.magnetic_field_covariance[0] = 3.2e-7;
    msg.mag_raw.magnetic_field_covariance[1] = 1e-7;
    msg.mag_raw.magnetic_field_covariance[2] = 1e-7;
    msg.mag_raw.magnetic_field_covariance[3] = 1e-7;
    msg.mag_raw.magnetic_field_covariance[4] = 3.2e-7;
    msg.mag_raw.magnetic_field_covariance[5] = 1e-7;
    msg.mag_raw.magnetic_field_covariance[6] = 1e-7;
    msg.mag_raw.magnetic_field_covariance[7] = 1e-7;
    msg.mag_raw.magnetic_field_covariance[8] = 4.1e-7;

    msg.mag_cal.magnetic_field_covariance[0] = 3.2e-7;
    msg.mag_cal.magnetic_field_covariance[1] = 1e-7;
    msg.mag_cal.magnetic_field_covariance[2] = 1e-7;
    msg.mag_cal.magnetic_field_covariance[3] = 1e-7;
    msg.mag_cal.magnetic_field_covariance[4] = 3.2e-7;
    msg.mag_cal.magnetic_field_covariance[5] = 1e-7;
    msg.mag_cal.magnetic_field_covariance[6] = 1e-7;
    msg.mag_cal.magnetic_field_covariance[7] = 1e-7;
    msg.mag_cal.magnetic_field_covariance[8] = 4.1e-7;

    mag_pub->publish(msg);
}

void BRoCoPublisher::handlePotentiometerPacket(uint8_t senderID, PotentiometerPacket* packet) {
    auto msg = avionics_interfaces::msg::AngleArray();

    msg.id = packet->id;

    for (uint8_t i = 0; i < msg.angles.size(); ++i)
        msg.angles[i] = packet->angles[i];

    potentiometer_pub->publish(msg);
}

void BRoCoPublisher::handleSpectroPacket(uint8_t senderID, SpectroResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::SpectroResponse();

    msg.id = packet->id;

    for (uint8_t i = 0; i < msg.data.size(); ++i)
        msg.data[i] = packet->data[i];

    msg.success = packet->success;

    spectro_response_pub->publish(msg);
}

void BRoCoPublisher::handleLaserPacket(uint8_t senderID, LaserResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::LaserResponse();

    msg.id = packet->id;

    msg.success = packet->success;

    laser_response_pub->publish(msg);
}

void BRoCoPublisher::handleServoPacket(uint8_t senderID, ServoResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::ServoResponse();

    msg.id = packet->id;

    msg.channel = packet->channel;
    msg.angle = packet->angle;
    msg.success = packet->success;

    servo_response_pub->publish(msg);
}

void BRoCoPublisher::handleLEDPacket(uint8_t senderID, LEDResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::LEDResponse();

    msg.id = packet->id;

    msg.state = packet->state;
    msg.success = packet->success;

    led_response_pub->publish(msg);
}

void BRoCoPublisher::handleMassConfigReqPacket(uint8_t senderID, MassConfigRequestPacket* packet) {
    auto msg = avionics_interfaces::msg::MassConfigRequestMCU();

    msg.id = packet->id;

    msg.req_offset = packet->req_offset;
    msg.req_scale = packet->req_scale;
    msg.req_alpha = packet->req_alpha;
    msg.req_channels_status = packet->req_channels_status;

    mass_config_req_pub->publish(msg);
}

void BRoCoPublisher::handleMassConfigPacket(uint8_t senderID, MassConfigResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::MassConfigResponse();

    msg.id = packet->id;

    msg.remote_command = packet->remote_command;
    msg.set_offset = packet->set_offset;
    msg.set_scale = packet->set_scale;
    msg.set_alpha = packet->set_alpha;
    msg.set_channels_status = packet->set_channels_status;
    msg.success = packet->success;

    for (uint8_t i = 0; i < 4; ++i) {
        msg.offset[i] = packet->offset[i];
        msg.scale[i] = packet->scale[i];
        msg.enabled_channels[i] = packet->enabled_channels[i];
    }
    msg.alpha = packet->alpha;

    // Set parameters inside parameter server
    std::string sensor;
    bool valid_id = false;
    if (packet->id == get_node_id("SC_DRILL_NODE_ID")) {
        sensor == "mass_drill";
        valid_id = true;
    } else if (packet->id == get_node_id("SC_CONTAINER_ID")) {
        sensor == "mass_container";
        valid_id = true;
    } else {
        RCLCPP_INFO(parent->get_logger(), "Mass config packet received but ID is not valid. Not saving parameters to parameter server.");
        valid_id = false;
    }
    if (valid_id) {
        std::vector<double> offset_vector(packet->offset, packet->offset + sizeof(packet->offset) / sizeof(packet->offset[0]));
        std::vector<double> scale_vector(packet->scale, packet->scale + sizeof(packet->scale) / sizeof(packet->scale[0]));
        std::vector<bool> enabled_channels_vector(packet->enabled_channels, packet->enabled_channels + sizeof(packet->enabled_channels) / sizeof(packet->enabled_channels[0]));
        set_param_calib(sensor, "offset", offset_vector);
        set_param_calib(sensor, "scale", scale_vector);
        set_param_calib(sensor, "alpha", packet->alpha);
        set_param_calib(sensor, "enabled_channels", enabled_channels_vector);
    }

    mass_config_response_pub->publish(msg);
}

uint32_t BRoCoPublisher::get_node_id(std::string node_name) {
    return dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
}


void BRoCoPublisher::set_destination_id(std::string node_name) {
    uint32_t id = dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

void BRoCoPublisher::set_destination_id(uint32_t id) {
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

std::string BRoCoPublisher::get_prefix() {
    return dynamic_cast<BRoCoManager*>(parent)->get_prefix();
}

std::string BRoCoPublisher::get_bus() {
    return dynamic_cast<BRoCoManager*>(parent)->get_bus();
}

template <typename T>
T BRoCoPublisher::get_param(const std::string& parameter_name) {
    return dynamic_cast<BRoCoManager*>(parent)->get_param<T>(parameter_name);
}

template <typename T>
void BRoCoPublisher::set_param_calib(const std::string& sensor, const std::string& parameter_name, const T& value) {
    dynamic_cast<BRoCoManager*>(parent)->set_param_calib(sensor, parameter_name, value);
}