#include "BRoCoSubscriber.h"
#include "BRoCo/CANBus.h"
#include "Time.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
uint32_t seq = 0;

BRoCoSubscriber::BRoCoSubscriber(std::shared_ptr<CANBus> bus, CanSocketDriver* driver, rclcpp::Node* parent) : bus(bus), driver(driver) {
  this->clk = parent->get_clock();
  this->timer = parent->create_wall_timer(1000ms, std::bind(&BRoCoSubscriber::callback, this));
}

void BRoCoSubscriber::callback() {
  static PingPacket packet;
  this->driver->TxFrameConfig(0x200);
  bus->send(&packet);
}