#ifndef BROCO_PUBLISHER_H
#define BROCO_PUBLISHER_H

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include "BRoCo/IODriver.h"
#include "BRoCo/IOBus.h"
#include "Protocol/Protocol.h"

using namespace std::chrono_literals;

class BRoCoPublisher {
public:
    BRoCoPublisher(IOBus* bus, rclcpp::Node* parent);

private:
    IOBus* bus;
    rclcpp::Clock::SharedPtr clk;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::TimerBase::SharedPtr timer;

    void timerCallback();

    void handleFourInOnePacket(uint8_t senderID, FOURINONEPacket* packet);
    void handleNPKPacket(uint8_t senderID, NPKPacket* packet);
    void handleVoltmeterPacket(uint8_t senderID, VoltmeterPacket* packet);
    void handleMassPacket(uint8_t senderID, MassPacket* packet);
    void handleIMUPacket(uint8_t senderID, IMUPacket* packet);
    void handlePotentiometerPacket(uint8_t senderID, PotentiometerPacket* packet);
    void handleSpectroPacket(uint8_t senderID, SpectroResponsePacket* packet);
    void handleLaserPacket(uint8_t senderID, LaserResponsePacket* packet);
    void handleServoPacket(uint8_t senderID, ServoResponsePacket* packet);
    void handleLEDPacket(uint8_t senderID, LEDResponsePacket* packet);

};

#endif /* BROCO_PUBLISHER_H */