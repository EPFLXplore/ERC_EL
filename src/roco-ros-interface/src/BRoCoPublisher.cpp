#include "BRoCoPublisher.h"
#include "Time.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <functional>

using namespace std::chrono_literals;

BRoCoPublisher::BRoCoPublisher(std::shared_ptr<CANBus> bus, rclcpp::Node* parent) : bus(bus) {
  auto node = rclcpp::Node::make_shared("brocopublisher");
  RCLCPP_INFO(node->get_logger(), "Adding handles...");

  this->clk = parent->get_clock();
  this->imu_pub = parent->create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", 10);
  bus->handle<IMUPacket>(std::bind(&BRoCoPublisher::handleIMUPacket, this, std::placeholders::_1, std::placeholders::_2));
}

void BRoCoPublisher::handleIMUPacket(uint8_t senderID, IMUPacket* packet) {
  // auto node = rclcpp::Node::make_shared("handleimu");
  // RCLCPP_INFO(node->get_logger(), "handle IMU called");
  auto msg = sensor_msgs::msg::Imu();

  msg.header.stamp = clk->now();

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