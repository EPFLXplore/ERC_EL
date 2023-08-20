/*
 * BRoCoManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "BRoCoManager.h"


BRoCoManager::BRoCoManager() : Node("broco_manager") {

    // Declare a parameter to hold the bus name
    this->declare_parameter("bus");

    if (!this->get_parameter("bus", bus_name)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get the 'bus' parameter");
    }
    RCLCPP_INFO(this->get_logger(), "Selected CAN bus: %s", bus_name.c_str());

    this->declare_parameter("topic_prefix");
    if (!this->get_parameter("topic_prefix", prefix)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get the 'namespace' parameter");
    }
    prefix = "/" + prefix;
    RCLCPP_INFO(this->get_logger(), "Namespace: %s", prefix.c_str());

    // Load parameters
    // Node IDs
    this->declare_parameter("JETSON_NODE_ID");
    this->declare_parameter("SC_CONTAINER_NODE_ID");
    this->declare_parameter("SC_DRILL_NODE_ID");
    this->declare_parameter("NAV_NODE_ID");
    this->declare_parameter("HD_NODE_ID");
    this->declare_parameter("GENERAL_NODE_ID");
    this->declare_parameter("MAX_NUMBER_NODES");

    // Ping and retry parameters
    this->declare_parameter("NODE_PING_INTERVAL");
    this->declare_parameter("NODE_STATE_WATCHDOG_TIMEOUT");
    this->declare_parameter("NODE_STATE_PUBLISH_INTERVAL");

    this->declare_parameter("CONNECTION_RETRY_NUM");
    this->declare_parameter("CONNECTION_RETRY_INTERVAL");
    this->declare_parameter("ATTEMPT_RETRY");

    // Topic names
    this->declare_parameter("FOUR_IN_ONE_TOPIC");
    this->declare_parameter("NPK_TOPIC");
    this->declare_parameter("VOLTAGE_TOPIC");
    this->declare_parameter("DRILL_MASS_TOPIC");
    this->declare_parameter("CONTAINER_MASS_TOPIC");
    this->declare_parameter("IMU_TOPIC");
    this->declare_parameter("POTENTIOMETER_TOPIC");
    this->declare_parameter("SPECTRO_TOPIC");
    this->declare_parameter("LASER_TOPIC");
    this->declare_parameter("SERVO_TOPIC");
    this->declare_parameter("LED_TOPIC");
    this->declare_parameter("NODE_STATE_TOPIC");

    this->declare_parameter("SPECTRO_REQ_TOPIC");
    this->declare_parameter("SERVO_REQ_TOPIC");
    this->declare_parameter("LASER_REQ_TOPIC");
    this->declare_parameter("LED_REQ_TOPIC");

    if (get_param<bool>("ATTEMPT_RETRY") == true) {
        // Create a timer to periodically attempt connection
        retry_timer = this->create_wall_timer(
            std::chrono::milliseconds(get_param<uint32_t>("CONNECTION_RETRY_INTERVAL")),
            std::bind(&BRoCoManager::retryConnection, this)
        );
    } else {
        // Create CanSocketDriver instance
        this->driver = new CanSocketDriver(bus_name.c_str());
        RCLCPP_INFO(this->get_logger(), "CAN driver connected on " + bus_name);
        createPubSub();
    }
}

BRoCoManager::~BRoCoManager() {
    RCLCPP_INFO(this->get_logger(), "Deleting sub and pub");
    delete this->sub;
    delete this->pub;
    delete this->bus;
    delete this->driver;
}

void BRoCoManager::retryConnection() {

    // Check if maximum retry attempts have been reached
    if (retry_count >= get_param<uint32_t>("CONNECTION_RETRY_NUM")) {
        RCLCPP_ERROR(this->get_logger(), "Maximum retry attempts reached. Giving up.");
        retry_timer->cancel();  // Stop the retry attempts
        rclcpp::shutdown();
        return;
    }

    // Create CanSocketDriver instance
    this->driver = new CanSocketDriver(bus_name.c_str());

    // Check if the driver is connected
    if (driver->isConnected()) {
        RCLCPP_INFO(this->get_logger(), "CAN driver connected on " + bus_name);
        retry_timer->cancel();  // Stop the retry attempts
        createPubSub();
    } else {
        ++retry_count;
        RCLCPP_WARN(this->get_logger(), "CAN driver not connected on '" + bus_name + "', retrying... (Attempt %d/%d)", 
            retry_count, get_param<uint32_t>("CONNECTION_RETRY_NUM"));
        delete driver;  // Clean up the previous instance
        driver = nullptr;
    }
}

std::string BRoCoManager::get_prefix() const {
    return prefix;
}

std::string BRoCoManager::get_bus() const {
    return bus_name;
}

void BRoCoManager::createPubSub() {
    this->bus = new CANBus(this->driver);
    this->pub = new BRoCoPublisher(this->bus, this);
    this->sub = new BRoCoSubscriber(this->bus, this);
    this->driver->start_reception();
}