/*
 * ConfigManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "ConfigManager.h"

using std::placeholders::_1;

ConfigManager::ConfigManager() : Node("config_manager") {
    
    this->declare_parameter("JETSON_NODE_ID");
    this->declare_parameter("SC_CONTAINER_NODE_ID");
    this->declare_parameter("SC_DRILL_NODE_ID");
    this->declare_parameter("NAV_NODE_ID");
    this->declare_parameter("HD_NODE_ID");
    this->declare_parameter("GENERAL_NODE_ID");
    this->declare_parameter("MAX_NUMBER_NODES");

    // Topic names =================================
    // Publishers
    this->declare_parameter("MASS_CONFIG_REQ_MCU_TOPIC");
    this->declare_parameter("MASS_CONFIG_TOPIC");

    // Subscribers
    this->declare_parameter("MASS_CONFIG_REQ_JETSON_TOPIC");

    // Calibration parameters ======================
    this->declare_parameter("mass_drill.offset");
    this->declare_parameter("mass_drill.scale");
    this->declare_parameter("mass_drill.alpha");
    this->declare_parameter("mass_drill.enabled_channels");

    this->declare_parameter("mass_container.offset");
    this->declare_parameter("mass_container.scale");
    this->declare_parameter("mass_container.alpha");
    this->declare_parameter("mass_container.enabled_channels");

    this->mass_config_req_pub = this->create_publisher<avionics_interfaces::msg::MassConfigRequestJetson>(get_param<std::string>("MASS_CONFIG_REQ_JETSON_TOPIC"), 10);
    this->mass_config_response_sub = this->create_subscription<avionics_interfaces::msg::MassConfigResponse>
        (get_param<std::string>("MASS_CONFIG_TOPIC"), 10, std::bind(&ConfigManager::massConfigResponseCallback, this, _1));
    this->mass_config_req_sub = this->create_subscription<avionics_interfaces::msg::MassConfigRequestMCU>
        (get_param<std::string>("MASS_CONFIG_REQ_MCU_TOPIC"), 10, std::bind(&ConfigManager::massConfigReqCallback, this, _1));

}

ConfigManager::~ConfigManager() {
    RCLCPP_INFO(this->get_logger(), "Deleting Config Manager");
}

void ConfigManager::massConfigReqCallback(const avionics_interfaces::msg::MassConfigRequestMCU::SharedPtr msg) {
    auto msg_req = avionics_interfaces::msg::MassConfigRequestJetson();
    msg_req.destination_id = msg->id;
    msg_req.remote_command = false; // request from MCU (and not from Jetson) -> false
    msg_req.set_offset = msg->req_offset;
    msg_req.set_scale = msg->req_scale;
    msg_req.set_alpha = msg->req_alpha;
    msg_req.set_channels_status = msg->req_channels_status;

    std::string sensor;
    bool valid_id = false;
    if (msg->id == get_param<uint32_t>("SC_CONTAINER_NODE_ID")) {
        sensor = "mass_container";
        valid_id = true;
    }
    else if (msg->id == get_param<uint32_t>("SC_DRILL_NODE_ID")) {
        sensor = "mass_drill";
        valid_id = true;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Config request received from MCU but ID is invalid. Not sending configuration.");
        valid_id = false;
    }

    if (valid_id) {
        std::vector<double> offset_double = get_param<std::vector<double>>(sensor + ".offset");
        std::vector<double> scale_double = get_param<std::vector<double>>(sensor + ".scale");
        
        for (uint8_t i = 0; i < 4; ++i) {
            msg_req.offset[i] = offset_double[i];
            msg_req.scale[i] = scale_double[i];
            msg_req.enabled_channels[i] = get_param<std::vector<bool>>(sensor + ".enabled_channels")[i];
        }
        msg_req.alpha =  static_cast<float>(get_param<double>(sensor + ".alpha"));
        
        mass_config_req_pub->publish(msg_req);
    }
}

void ConfigManager::massConfigResponseCallback(const avionics_interfaces::msg::MassConfigResponse::SharedPtr msg) {

    std::string sensor;
    bool valid_id = false;
    if (msg->id == get_param<uint32_t>("SC_CONTAINER_NODE_ID")) {
        sensor = "mass_container";
        valid_id = true;
    }
    else if (msg->id == get_param<uint32_t>("SC_DRILL_NODE_ID")) {
        sensor = "mass_drill";
        valid_id = true;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Config response received but ID is invalid. Not saving config to YAML file.");
        valid_id = false;
    }

    if (valid_id) {
        // std::vector<double> offset_double = get_param<std::vector<double>>(sensor + ".offset");
        RCLCPP_INFO(this->get_logger(), "Saving mass calibration parameters into YAML file...");
        std::vector<double> offset_vector;
        std::vector<double> scale_vector;
        std::vector<bool> enabled_channels_vector;
        for (uint8_t i = 0; i < 4; i++) {
            offset_vector.push_back(static_cast<double>(msg->offset[i]));
            scale_vector.push_back(static_cast<double>(msg->scale[i]));
            enabled_channels_vector.push_back(msg->enabled_channels[i]);
        }

        // Set parameters into YAML file
        if (msg->set_alpha)
            set_param_calib(sensor, "alpha", msg->alpha);
        if (msg->set_offset)
            set_param_calib(sensor, "offset", offset_vector);
        if (msg->set_scale)
            set_param_calib(sensor, "scale", scale_vector);
        if (msg->set_channels_status)
            set_param_calib(sensor, "enabled_channels", enabled_channels_vector);
    }
}