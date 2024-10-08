/*
 * BRoCoPublisher.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef BROCO_PUBLISHER_H
#define BROCO_PUBLISHER_H

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "avionics_interfaces/msg/four_in_one.hpp"
#include "avionics_interfaces/msg/npk.hpp"
#include "avionics_interfaces/msg/voltage.hpp"
#include "avionics_interfaces/msg/mass_array.hpp"
#include "avionics_interfaces/msg/imu.hpp"
#include "avionics_interfaces/msg/mag.hpp"
#include "avionics_interfaces/msg/angle_array.hpp"
#include "avionics_interfaces/msg/spectro_response.hpp"
#include "avionics_interfaces/msg/laser_response.hpp"
#include "avionics_interfaces/msg/servo_response.hpp"
#include "avionics_interfaces/msg/led_response.hpp"
#include "avionics_interfaces/msg/node_state_array.hpp"

// config messages
#include "avionics_interfaces/msg/mass_config_request_mcu.hpp"
#include "avionics_interfaces/msg/mass_config_response.hpp"

#include "avionics_interfaces/msg/pot_config_request_mcu.hpp"
#include "avionics_interfaces/msg/pot_config_response.hpp"

#include "avionics_interfaces/msg/servo_config_request_mcu.hpp"
#include "avionics_interfaces/msg/servo_config_response.hpp"

#include "avionics_interfaces/msg/accel_config_request_mcu.hpp"
#include "avionics_interfaces/msg/accel_config_response.hpp"

#include "avionics_interfaces/msg/gyro_config_request_mcu.hpp"
#include "avionics_interfaces/msg/gyro_config_response.hpp"

#include "avionics_interfaces/msg/mag_config_request_mcu.hpp"
#include "avionics_interfaces/msg/mag_config_response.hpp"

#include "BRoCo/CANBus.h"
#include "Protocol/Protocol.h"

using namespace std::chrono_literals;

class BRoCoPublisher {
public:
    BRoCoPublisher(CANBus* bus, rclcpp::Node* parent);

private:
    CANBus* bus;
    rclcpp::Node* parent;
    rclcpp::Clock::SharedPtr clk;
    std::vector<bool> node_state;

    // Publishers
    rclcpp::Publisher<avionics_interfaces::msg::FourInOne>::SharedPtr four_in_one_pub;
    rclcpp::Publisher<avionics_interfaces::msg::NPK>::SharedPtr npk_pub;
    rclcpp::Publisher<avionics_interfaces::msg::Voltage>::SharedPtr voltage_pub;
    rclcpp::Publisher<avionics_interfaces::msg::MassArray>::SharedPtr drill_mass_pub;
    rclcpp::Publisher<avionics_interfaces::msg::MassArray>::SharedPtr container_mass_pub;
    rclcpp::Publisher<avionics_interfaces::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<avionics_interfaces::msg::Mag>::SharedPtr mag_pub;
    rclcpp::Publisher<avionics_interfaces::msg::AngleArray>::SharedPtr potentiometer_pub;
    rclcpp::Publisher<avionics_interfaces::msg::SpectroResponse>::SharedPtr spectro_response_pub;
    rclcpp::Publisher<avionics_interfaces::msg::LaserResponse>::SharedPtr laser_response_pub;
    rclcpp::Publisher<avionics_interfaces::msg::ServoResponse>::SharedPtr servo_response_pub;
    rclcpp::Publisher<avionics_interfaces::msg::LEDResponse>::SharedPtr led_response_pub;

    rclcpp::Publisher<avionics_interfaces::msg::NodeStateArray>::SharedPtr node_state_pub;

    rclcpp::Publisher<avionics_interfaces::msg::MassConfigRequestMCU>::SharedPtr mass_config_req_pub;
    rclcpp::Publisher<avionics_interfaces::msg::MassConfigResponse>::SharedPtr mass_config_response_pub;

    rclcpp::Publisher<avionics_interfaces::msg::PotConfigRequestMCU>::SharedPtr pot_config_req_pub;
    rclcpp::Publisher<avionics_interfaces::msg::PotConfigResponse>::SharedPtr pot_config_response_pub;

    rclcpp::Publisher<avionics_interfaces::msg::ServoConfigRequestMCU>::SharedPtr servo_config_req_pub;
    rclcpp::Publisher<avionics_interfaces::msg::ServoConfigResponse>::SharedPtr servo_config_response_pub;

    rclcpp::Publisher<avionics_interfaces::msg::AccelConfigRequestMCU>::SharedPtr accel_config_req_pub;
    rclcpp::Publisher<avionics_interfaces::msg::AccelConfigResponse>::SharedPtr accel_config_response_pub;

    rclcpp::Publisher<avionics_interfaces::msg::GyroConfigRequestMCU>::SharedPtr gyro_config_req_pub;
    rclcpp::Publisher<avionics_interfaces::msg::GyroConfigResponse>::SharedPtr gyro_config_response_pub;

    rclcpp::Publisher<avionics_interfaces::msg::MagConfigRequestMCU>::SharedPtr mag_config_req_pub;
    rclcpp::Publisher<avionics_interfaces::msg::MagConfigResponse>::SharedPtr mag_config_response_pub;

    rclcpp::TimerBase::SharedPtr timer;
    std::vector<rclcpp::TimerBase::SharedPtr> watchdog_timers;
    rclcpp::TimerBase::SharedPtr node_state_pub_timer;

    template <typename T>
    T get_param(const std::string& parameter_name);

    template <typename T>
    void set_param_calib(const std::string& sensor, const std::string& parameter_name, const T& value);

    uint32_t get_node_id(std::string node_name);
    void set_destination_id(uint16_t id);
    void set_destination_id(std::string node_name);

    std::string get_prefix();
    std::string get_bus();

    // Callbacks
    void timerPingCallback();
    void nodeStateCallback();
    void watchdogCallback(size_t nodeID);

    // RoCo callbacks
    void handleFourInOnePacket(uint8_t senderID, FOURINONEPacket* packet);
    void handleNPKPacket(uint8_t senderID, NPKPacket* packet);
    void handleVoltmeterPacket(uint8_t senderID, VoltmeterPacket* packet);
    void handleMassPacket(uint8_t senderID, MassPacket* packet);
    void handleIMUPacket(uint8_t senderID, IMUPacket* packet);
    void handleMagPacket(uint8_t senderID, MagPacket* packet);
    void handlePotentiometerPacket(uint8_t senderID, PotentiometerPacket* packet);
    void handleSpectroPacket(uint8_t senderID, SpectroResponsePacket* packet);
    void handleLaserPacket(uint8_t senderID, LaserResponsePacket* packet);
    void handleServoPacket(uint8_t senderID, ServoResponsePacket* packet);
    void handleLEDPacket(uint8_t senderID, LEDResponsePacket* packet);
    void handlePingPacket(uint8_t senderID, PingPacket* packet);

    void handleMassConfigReqPacket(uint8_t senderID, MassConfigRequestPacket* packet);
    void handleMassConfigPacket(uint8_t senderID, MassConfigResponsePacket* packet);

    void handlePotConfigReqPacket(uint8_t senderID, PotentiometerConfigRequestPacket* packet);
    void handlePotConfigPacket(uint8_t senderID, PotentiometerConfigResponsePacket* packet);

    void handleServoConfigReqPacket(uint8_t senderID, ServoConfigRequestPacket* packet);
    void handleServoConfigPacket(uint8_t senderID, ServoConfigResponsePacket* packet);

    void handleAccelConfigReqPacket(uint8_t senderID, AccelConfigRequestPacket* packet);
    void handleAccelConfigPacket(uint8_t senderID, AccelConfigResponsePacket* packet);

    void handleGyroConfigReqPacket(uint8_t senderID, GyroConfigRequestPacket* packet);
    void handleGyroConfigPacket(uint8_t senderID, GyroConfigResponsePacket* packet);

    void handleMagConfigReqPacket(uint8_t senderID, MagConfigRequestPacket* packet);
    void handleMagConfigPacket(uint8_t senderID, MagConfigResponsePacket* packet);
};

#endif /* BROCO_PUBLISHER_H */