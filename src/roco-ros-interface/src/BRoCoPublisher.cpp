/*
 * BRoCoPublisher.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoPublisher.h"
#include "BRoCoManager.h"

#include "Utils.h"

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

    this->pot_config_req_pub = parent->create_publisher<avionics_interfaces::msg::PotConfigRequestMCU>(get_prefix() + get_param<std::string>("POT_CONFIG_REQ_MCU_TOPIC"), 10);
    this->pot_config_response_pub = parent->create_publisher<avionics_interfaces::msg::PotConfigResponse>(get_prefix() + get_param<std::string>("POT_CONFIG_TOPIC"), 10);

    this->servo_config_req_pub = parent->create_publisher<avionics_interfaces::msg::ServoConfigRequestMCU>(get_prefix() + get_param<std::string>("SERVO_CONFIG_REQ_MCU_TOPIC"), 10);
    this->servo_config_response_pub = parent->create_publisher<avionics_interfaces::msg::ServoConfigResponse>(get_prefix() + get_param<std::string>("SERVO_CONFIG_TOPIC"), 10);

    this->accel_config_req_pub = parent->create_publisher<avionics_interfaces::msg::AccelConfigRequestMCU>(get_prefix() + get_param<std::string>("ACCEL_CONFIG_REQ_MCU_TOPIC"), 10);
    this->accel_config_response_pub = parent->create_publisher<avionics_interfaces::msg::AccelConfigResponse>(get_prefix() + get_param<std::string>("ACCEL_CONFIG_TOPIC"), 10);

    this->gyro_config_req_pub = parent->create_publisher<avionics_interfaces::msg::GyroConfigRequestMCU>(get_prefix() + get_param<std::string>("GYRO_CONFIG_REQ_MCU_TOPIC"), 10);
    this->gyro_config_response_pub = parent->create_publisher<avionics_interfaces::msg::GyroConfigResponse>(get_prefix() + get_param<std::string>("GYRO_CONFIG_TOPIC"), 10);

    this->mag_config_req_pub = parent->create_publisher<avionics_interfaces::msg::MagConfigRequestMCU>(get_prefix() + get_param<std::string>("MAG_CONFIG_REQ_MCU_TOPIC"), 10);
    this->mag_config_response_pub = parent->create_publisher<avionics_interfaces::msg::MagConfigResponse>(get_prefix() + get_param<std::string>("MAG_CONFIG_TOPIC"), 10);

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

    bus->handle<PotentiometerConfigRequestPacket>(std::bind(&BRoCoPublisher::handlePotConfigReqPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<PotentiometerConfigResponsePacket>(std::bind(&BRoCoPublisher::handlePotConfigPacket, this, std::placeholders::_1, std::placeholders::_2));

    bus->handle<ServoConfigRequestPacket>(std::bind(&BRoCoPublisher::handleServoConfigReqPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<ServoConfigResponsePacket>(std::bind(&BRoCoPublisher::handleServoConfigPacket, this, std::placeholders::_1, std::placeholders::_2));

    bus->handle<AccelConfigRequestPacket>(std::bind(&BRoCoPublisher::handleAccelConfigReqPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<AccelConfigResponsePacket>(std::bind(&BRoCoPublisher::handleAccelConfigPacket, this, std::placeholders::_1, std::placeholders::_2));

    bus->handle<GyroConfigRequestPacket>(std::bind(&BRoCoPublisher::handleGyroConfigReqPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<GyroConfigResponsePacket>(std::bind(&BRoCoPublisher::handleGyroConfigPacket, this, std::placeholders::_1, std::placeholders::_2));

    bus->handle<MagConfigRequestPacket>(std::bind(&BRoCoPublisher::handleMagConfigReqPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<MagConfigResponsePacket>(std::bind(&BRoCoPublisher::handleMagConfigPacket, this, std::placeholders::_1, std::placeholders::_2));

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
    uint16_t id = packet->id;
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

    // Convert from scaled uint16_to float
    for (uint8_t i = 0; i < msg.data.size(); ++i)
        msg.data[i] = scaledUInt16ToFloat(packet->data[i], packet->max_val);

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
        sensor = "mass_drill";
        valid_id = true;
    } else if (packet->id == get_node_id("SC_CONTAINER_ID")) {
        sensor = "mass_container";
        valid_id = true;
    } else {
        RCLCPP_INFO(parent->get_logger(), "Mass config packet received but ID is not valid. " 
            "Not saving parameters to parameter server.");
        valid_id = false;
    }
    if (valid_id) {
        if (packet->set_offset) {
            std::vector<double> offset_vector(packet->offset, packet->offset 
                + sizeof(packet->offset) / sizeof(packet->offset[0]));
            set_param_calib(sensor, "offset", offset_vector);
        }
        if (packet->set_scale) {
            std::vector<double> scale_vector(packet->scale, packet->scale 
                + sizeof(packet->scale) / sizeof(packet->scale[0]));
            set_param_calib(sensor, "scale", scale_vector);
        }
        if (packet->set_channels_status) {
            std::vector<bool> enabled_channels_vector(packet->enabled_channels, packet->enabled_channels 
                + sizeof(packet->enabled_channels) / sizeof(packet->enabled_channels[0]));
            set_param_calib(sensor, "enabled_channels", enabled_channels_vector);

        }
        if (packet->set_alpha)
            set_param_calib(sensor, "alpha", packet->alpha);
    }

    mass_config_response_pub->publish(msg);
}

void BRoCoPublisher::handlePotConfigReqPacket(uint8_t senderID, PotentiometerConfigRequestPacket* packet) {
    auto msg = avionics_interfaces::msg::PotConfigRequestMCU();

    msg.id = packet->id;

    msg.req_min_voltages = packet->req_min_voltages;
    msg.req_max_voltages = packet->req_max_voltages;
    msg.req_min_angles = packet->req_min_angles;
    msg.req_max_angles = packet->req_max_angles;
    msg.req_channels_status = packet->req_channels_status;

    pot_config_req_pub->publish(msg);
}

void BRoCoPublisher::handlePotConfigPacket(uint8_t senderID, PotentiometerConfigResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::PotConfigResponse();

    msg.id = packet->id;

    msg.remote_command = packet->remote_command;
    msg.set_min_voltages = packet->set_min_voltages;
    msg.set_max_voltages = packet->set_max_voltages;
    msg.set_min_angles = packet->set_min_angles;
    msg.set_max_angles = packet->set_max_angles;
    msg.set_channels_status = packet->set_channels_status;
    msg.success = packet->success;

    for (uint8_t i = 0; i < 4; ++i) {
        msg.min_voltages[i] = scaledUInt16ToFloat(packet->min_voltages[i], packet->min_voltages_max_val);
        msg.max_voltages[i] = scaledUInt16ToFloat(packet->max_voltages[i], packet->max_voltages_max_val);
        msg.min_angles[i] = scaledUInt16ToFloat(packet->min_angles[i], packet->min_angles_max_val);
        msg.max_angles[i] = scaledUInt16ToFloat(packet->max_angles[i], packet->max_angles_max_val);
        msg.enabled_channels[i] = packet->enabled_channels[i];
    }

    std::string sensor = "potentiometer";

    std::vector<double> min_voltages_vector;
    std::vector<double> max_voltages_vector;
    std::vector<double> min_angles_vector;
    std::vector<double> max_angles_vector;
    for (int i = 0; i < 4; ++i) {
        min_voltages_vector.push_back(static_cast<double>(msg.min_voltages[i]));
        max_voltages_vector.push_back(static_cast<double>(msg.max_voltages[i]));
        min_angles_vector.push_back(static_cast<double>(msg.min_angles[i]));
        max_angles_vector.push_back(static_cast<double>(msg.max_angles[i]));
    }

    if (packet->set_min_voltages) 
        set_param_calib(sensor, "min_voltages", min_voltages_vector);
    
    if (packet->set_max_voltages) 
        set_param_calib(sensor, "max_voltages", max_voltages_vector);
    
    if (packet->set_min_angles) 
        set_param_calib(sensor, "min_angles", min_angles_vector);
    
    if (packet->set_max_angles) 
        set_param_calib(sensor, "max_angles", max_angles_vector);
    
    if (packet->set_channels_status) {
        std::vector<bool> enabled_channels_vector(packet->enabled_channels, packet->enabled_channels 
            + sizeof(packet->enabled_channels) / sizeof(packet->enabled_channels[0]));
        set_param_calib(sensor, "enabled_channels", enabled_channels_vector);
    }

    pot_config_response_pub->publish(msg);
}

void BRoCoPublisher::handleServoConfigReqPacket(uint8_t senderID, ServoConfigRequestPacket* packet) {
    auto msg = avionics_interfaces::msg::ServoConfigRequestMCU();

    msg.id = packet->id;

    msg.req_min_duty = packet->req_min_duty;
    msg.req_max_duty = packet->req_max_duty;
    msg.req_min_angles = packet->req_min_angles;
    msg.req_max_angles = packet->req_max_angles;

    servo_config_req_pub->publish(msg);
}

void BRoCoPublisher::handleServoConfigPacket(uint8_t senderID, ServoConfigResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::ServoConfigResponse();

    msg.id = packet->id;

    msg.remote_command = packet->remote_command;
    msg.set_min_duty = packet->set_min_duty;
    msg.set_max_duty = packet->set_max_duty;
    msg.set_min_angles = packet->set_min_angles;
    msg.set_max_angles = packet->set_max_angles;
    msg.success = packet->success;

    for (uint8_t i = 0; i < 4; ++i) {
        msg.min_duty[i] = scaledUInt16ToFloat(packet->min_duty[i], packet->min_duty_max_val);
        msg.max_duty[i] = scaledUInt16ToFloat(packet->max_duty[i], packet->max_duty_max_val);
        msg.min_angles[i] = scaledUInt16ToFloat(packet->min_angles[i], packet->min_angles_max_val);
        msg.max_angles[i] = scaledUInt16ToFloat(packet->max_angles[i], packet->max_angles_max_val);
    }

    std::string sensor = "servo";

    std::vector<double> min_duty_vector;
    std::vector<double> max_duty_vector;
    std::vector<double> min_angles_vector;
    std::vector<double> max_angles_vector;
    for (int i = 0; i < 4; ++i) {
        min_duty_vector.push_back(static_cast<double>(msg.min_duty[i]));
        max_duty_vector.push_back(static_cast<double>(msg.max_duty[i]));
        min_angles_vector.push_back(static_cast<double>(msg.min_angles[i]));
        max_angles_vector.push_back(static_cast<double>(msg.max_angles[i]));
    }

    if (packet->set_min_duty) 
        set_param_calib(sensor, "min_duty", min_duty_vector);
    
    if (packet->set_max_duty) 
        set_param_calib(sensor, "max_duty", max_duty_vector);
    
    if (packet->set_min_angles) 
        set_param_calib(sensor, "min_angles", min_angles_vector);
    
    if (packet->set_max_angles) 
        set_param_calib(sensor, "max_angles", max_angles_vector);
    

    servo_config_response_pub->publish(msg);
}

void BRoCoPublisher::handleAccelConfigReqPacket(uint8_t senderID, AccelConfigRequestPacket* packet) {
    auto msg = avionics_interfaces::msg::AccelConfigRequestMCU();

    msg.id = packet->id;

    msg.req_bias = packet->req_bias;
    msg.req_transform = packet->req_transform;

    accel_config_req_pub->publish(msg);
}

void BRoCoPublisher::handleAccelConfigPacket(uint8_t senderID, AccelConfigResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::AccelConfigResponse();

    msg.id = packet->id;

    msg.remote_command = packet->remote_command;
    msg.set_bias = packet->set_bias;
    msg.set_transform = packet->set_transform;
    msg.success = packet->success;

    for (uint8_t i = 0; i < 4; ++i) {
        msg.bias[i] = packet->bias[i];
    }

    for (uint8_t i = 0; i < 9; ++i) {
        msg.transform[i] = packet->transform[i];
    }

    std::string sensor = "accel";

    if (packet->set_bias) {
        std::vector<double> bias_vector(packet->bias, packet->bias 
            + sizeof(packet->bias) / sizeof(packet->bias[0]));
        set_param_calib(sensor, "bias", bias_vector);
    }
    if (packet->set_transform) {
        std::vector<double> transform_vector(packet->transform, packet->transform 
            + sizeof(packet->transform) / sizeof(packet->transform[0]));
        set_param_calib(sensor, "transform", transform_vector);
    }

    accel_config_response_pub->publish(msg);
}

void BRoCoPublisher::handleGyroConfigReqPacket(uint8_t senderID, GyroConfigRequestPacket* packet) {
    auto msg = avionics_interfaces::msg::GyroConfigRequestMCU();

    msg.id = packet->id;

    msg.req_bias = packet->req_bias;

    gyro_config_req_pub->publish(msg);
}

void BRoCoPublisher::handleGyroConfigPacket(uint8_t senderID, GyroConfigResponsePacket* packet) {
    auto msg = avionics_interfaces::msg::GyroConfigResponse();

    msg.id = packet->id;

    msg.remote_command = packet->remote_command;
    msg.set_bias = packet->set_bias;
    msg.success = packet->success;

    for (uint8_t i = 0; i < 4; ++i) {
        msg.bias[i] = packet->bias[i];
    }

    std::string sensor = "gyro";

    if (packet->set_bias) {
        std::vector<double> bias_vector(packet->bias, packet->bias 
            + sizeof(packet->bias) / sizeof(packet->bias[0]));
        set_param_calib(sensor, "bias", bias_vector);
    }

    gyro_config_response_pub->publish(msg);
}

void BRoCoPublisher::handleMagConfigReqPacket(uint8_t senderID, MagConfigRequestPacket* packet) {
    auto msg = avionics_interfaces::msg::MagConfigRequestMCU();

    msg.id = packet->id;

    msg.req_hard_iron = packet->req_hard_iron;
    msg.req_soft_iron = packet->req_soft_iron;

    mag_config_req_pub->publish(msg);
}

void BRoCoPublisher::handleMagConfigPacket(uint8_t senderID, MagConfigResponsePacket* packet) {
    RCLCPP_INFO(parent->get_logger(), "Mag config response received from MCU");
    auto msg = avionics_interfaces::msg::MagConfigResponse();

    msg.id = packet->id;

    msg.remote_command = packet->remote_command;
    msg.set_hard_iron = packet->set_hard_iron;
    msg.set_soft_iron = packet->set_soft_iron;
    msg.success = packet->success;

    for (uint8_t i = 0; i < 4; ++i) {
        msg.hard_iron[i] = packet->hard_iron[i];
    }

    for (uint8_t i = 0; i < 9; ++i) {
        msg.soft_iron[i] = packet->soft_iron[i];
    }

    std::string sensor = "mag";

    if (packet->set_hard_iron) {
        std::vector<double> hard_iron_vector(packet->hard_iron, packet->hard_iron 
            + sizeof(packet->hard_iron) / sizeof(packet->hard_iron[0]));
        set_param_calib(sensor, "hard_iron", hard_iron_vector);
    }
    if (packet->set_soft_iron) {
        std::vector<double> soft_iron_vector(packet->soft_iron, packet->soft_iron 
            + sizeof(packet->soft_iron) / sizeof(packet->soft_iron[0]));
        set_param_calib(sensor, "soft_iron", soft_iron_vector);
    }

    mag_config_response_pub->publish(msg);
}



uint32_t BRoCoPublisher::get_node_id(std::string node_name) {
    return dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
}


void BRoCoPublisher::set_destination_id(std::string node_name) {
    uint16_t id = dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

void BRoCoPublisher::set_destination_id(uint16_t id) {
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig((uint32_t)id);
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