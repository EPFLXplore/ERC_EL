#include "BRoCoSubscriber.h"
#include "BRoCo/CanSocketDriver.h"
#include "Time.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
uint32_t seq = 0;

BRoCoSubscriber::BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent) : bus(bus), parent(parent) {
    this->clk = parent->get_clock();
    this->timer = parent->create_wall_timer(1000ms, std::bind(&BRoCoSubscriber::callback, this));

  // servo_request_server_ = this->parent->create_service<avionics_interfaces::srv::ServoRequest>(
  //         "servo_request", std::bind(&BRoCoSubscriber::handle_servo_request, this, std::placeholders::_1, std::placeholders::_2));

  // bus->handle<ServoResponsePacket>(std::bind(&BRoCoSubscriber::handleServoResponse, this, std::placeholders::_1, std::placeholders::_2));

}

void BRoCoSubscriber::callback() {
    static PingPacket packet;
    set_destination_id(0x7FF);
    bus->send(&packet);
}

void BRoCoSubscriber::set_destination_id(uint32_t id) {
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

// void BRoCoSubscriber::handleServoResponse(uint8_t senderID, ServoResponsePacket* packet) {
//     // Process the response packet as needed

//     // Set the packet_success_ based on your logic
//     packet_success_ = packet->success;

//     // Signal completion of response processing
//     response_promise_->set_value();
// }

// void BRoCoSubscriber::handle_servo_request(
//     const std::shared_ptr<avionics_interfaces::srv::ServoRequest::Request> request,
//     std::shared_ptr<avionics_interfaces::srv::ServoRequest::Response> response) {
    
//     // Create a ServoPacket instance
//     ServoPacket packet;
//     packet.channel = request->channel;
//     packet.angle = request->angle;

//     // Send the packet through the CAN bus to the STM32
//     bus->send(&packet);

//     response_promise_ = new std::promise<void>;

//     // Wait asynchronously for handleServoResponse to be called
//     response_future_ = response_promise_->get_future();
    
//     // Set the response success flag
//     response->success = true;

//     // Set the success response based on packet_success_ in handleServoResponse
//     if (response_future_.wait_for(std::chrono::milliseconds(10000)) == std::future_status::ready) {
//         response->success = packet_success_;
//     } else {
//         response->success = false; // Timeout, consider setting a proper error value
//     }
// }