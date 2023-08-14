/*
 * BRoCoPublisher.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoPublisher.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <functional>

using namespace std::chrono_literals;

BRoCoPublisher::BRoCoPublisher(CANBus* bus, rclcpp::Node* parent) : bus(bus) {
  auto node = rclcpp::Node::make_shared("brocopublisher");
  RCLCPP_INFO(node->get_logger(), "Adding handles...");

  this->clk = parent->get_clock();
  this->four_in_one_pub = parent->create_publisher<avionics_interfaces::msg::FourInOne>("/four_in_one", 10);
  this->npk_pub = parent->create_publisher<avionics_interfaces::msg::NPK>("/npk", 10);
  this->voltage_pub = parent->create_publisher<std_msgs::msg::Float32>("/voltage", 10);
  this->drill_mass_pub = parent->create_publisher<avionics_interfaces::msg::MassArray>("/drill/mass", 10);
  this->container_mass_pub = parent->create_publisher<avionics_interfaces::msg::MassArray>("container/mass", 10);
  this->imu_pub = parent->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  this->potentiometer_pub = parent->create_publisher<avionics_interfaces::msg::AngleArray>("/potentiometer", 10);
  this->spectro_response_pub = parent->create_publisher<avionics_interfaces::msg::SpectroResponse>("/spectro_response", 10);
  this->laser_response_pub = parent->create_publisher<avionics_interfaces::msg::LaserResponse>("/laser_response", 10);
  this->servo_response_pub = parent->create_publisher<avionics_interfaces::msg::ServoResponse>("/servo_response", 10);
  this->led_response_pub = parent->create_publisher<avionics_interfaces::msg::LEDResponse>("/led_response", 10);
  bus->handle<FOURINONEPacket>(std::bind(&BRoCoPublisher::handleFourInOnePacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<NPKPacket>(std::bind(&BRoCoPublisher::handleNPKPacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<VoltmeterPacket>(std::bind(&BRoCoPublisher::handleVoltmeterPacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<MassPacket>(std::bind(&BRoCoPublisher::handleMassPacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<IMUPacket>(std::bind(&BRoCoPublisher::handleIMUPacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<PotentiometerPacket>(std::bind(&BRoCoPublisher::handlePotentiometerPacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<SpectroResponsePacket>(std::bind(&BRoCoPublisher::handleSpectroPacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<LaserResponsePacket>(std::bind(&BRoCoPublisher::handleLaserPacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<ServoResponsePacket>(std::bind(&BRoCoPublisher::handleServoPacket, this, std::placeholders::_1, std::placeholders::_2));
  bus->handle<LEDResponsePacket>(std::bind(&BRoCoPublisher::handleLEDPacket, this, std::placeholders::_1, std::placeholders::_2));
}

void BRoCoPublisher::handleFourInOnePacket(uint8_t senderID, FOURINONEPacket* packet) {
  auto msg = avionics_interfaces::msg::FourInOne();

  msg.temperature = packet->temperature;
  msg.moisture = packet->moisture;
  msg.conductivity = packet->conductivity;
  msg.ph = packet->pH;

  four_in_one_pub->publish(msg);
}

void BRoCoPublisher::handleNPKPacket(uint8_t senderID, NPKPacket* packet) {
  auto msg = avionics_interfaces::msg::NPK();

  msg.nitrogen = packet->nitrogen;
  msg.phosphorus = packet->phosphorus;
  msg.potassium = packet->potassium;

  npk_pub->publish(msg);
}

void BRoCoPublisher::handleVoltmeterPacket(uint8_t senderID, VoltmeterPacket* packet) {
  auto msg = std_msgs::msg::Float32();

  msg.data = packet->voltage;

  voltage_pub->publish(msg);
}

void BRoCoPublisher::handleMassPacket(uint8_t senderID, MassPacket* packet) {
  auto msg = avionics_interfaces::msg::MassArray();

  for (uint8_t i = 0; i < 4; ++i)
    msg.mass[i] = packet->mass[i];

  if (packet->id == 0x001)
    drill_mass_pub->publish(msg);
  else if (packet->id == 0x002)
    container_mass_pub->publish(msg);
  else
    std::cout << "Mass received but ID is not valid" << std::endl;
}

void BRoCoPublisher::handleIMUPacket(uint8_t senderID, IMUPacket* packet) {
  auto msg = sensor_msgs::msg::Imu();

  msg.header.stamp = clk->now();
  msg.header.frame_id = "imu";

  msg.angular_velocity.x = packet->angular[0];
  msg.angular_velocity.y = packet->angular[1];
  msg.angular_velocity.z = packet->angular[2];
  msg.linear_acceleration.x = packet->acceleration[0];
  msg.linear_acceleration.y = packet->acceleration[1];
  msg.linear_acceleration.z = packet->acceleration[2];

  msg.orientation.w = packet->orientation[0];
  msg.orientation.x = packet->orientation[1];
  msg.orientation.y = packet->orientation[2];
  msg.orientation.z = packet->orientation[3];

  // To change

  msg.orientation_covariance[0] = 1e-4;
  msg.orientation_covariance[3] = 1e-4;
  msg.orientation_covariance[6] = 1e-4;

  msg.linear_acceleration_covariance[0] = 1.4e-3;
  msg.linear_acceleration_covariance[1] = 1.0e-4;
  msg.linear_acceleration_covariance[2] = 4.5e-5;
  msg.linear_acceleration_covariance[3] = 1.0e-4;
  msg.linear_acceleration_covariance[4] = 2.1e-3;
  msg.linear_acceleration_covariance[5] = 2.5e-4;
  msg.linear_acceleration_covariance[6] = 4.5e-5;
  msg.linear_acceleration_covariance[7] = 2.5e-4;
  msg.linear_acceleration_covariance[8] = 2.0e-3;

  msg.angular_velocity_covariance[0] = 1.6e-2;
  msg.angular_velocity_covariance[1] = -4.1e-5;
  msg.angular_velocity_covariance[2] = 4.7e-3;
  msg.angular_velocity_covariance[3] = -4.1e-5;
  msg.angular_velocity_covariance[4] = 1.7e-2;
  msg.angular_velocity_covariance[5] = 1.1e-4;
  msg.angular_velocity_covariance[6] = 4.7e-3;
  msg.angular_velocity_covariance[7] = 1.1e-4;
  msg.angular_velocity_covariance[8] = 2.5e-2;

  imu_pub->publish(msg);
}

void BRoCoPublisher::handlePotentiometerPacket(uint8_t senderID, PotentiometerPacket* packet) {
  auto msg = avionics_interfaces::msg::AngleArray();

  for (uint8_t i = 0; i < 4; ++i)
    msg.angles[i] = packet->angles[i];

  potentiometer_pub->publish(msg);
}
void BRoCoPublisher::handleSpectroPacket(uint8_t senderID, SpectroResponsePacket* packet) {
  auto msg = avionics_interfaces::msg::SpectroResponse();

  for (uint8_t i = 0; i < 18; ++i)
   msg.data[i] = packet->data[i];

  msg.success = packet->success;

  spectro_response_pub->publish(msg);
}

void BRoCoPublisher::handleLaserPacket(uint8_t senderID, LaserResponsePacket* packet) {
  auto msg = avionics_interfaces::msg::LaserResponse();

  msg.success = packet->success;

  laser_response_pub->publish(msg);
}

void BRoCoPublisher::handleServoPacket(uint8_t senderID, ServoResponsePacket* packet) {
  auto msg = avionics_interfaces::msg::ServoResponse();

  msg.channel = packet->channel;
  msg.angle = packet->angle;
  msg.success = packet->success;

  servo_response_pub->publish(msg);
}

void BRoCoPublisher::handleLEDPacket(uint8_t senderID, LEDResponsePacket* packet) {
  auto msg = avionics_interfaces::msg::LEDResponse();

  msg.state = packet->state;
  msg.success = packet->success;

  led_response_pub->publish(msg);
}