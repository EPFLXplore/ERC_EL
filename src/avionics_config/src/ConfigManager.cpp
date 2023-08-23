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
    this->declare_parameter("POT_CONFIG_REQ_MCU_TOPIC");
    this->declare_parameter("POT_CONFIG_TOPIC");
    this->declare_parameter("SERVO_CONFIG_REQ_MCU_TOPIC");
    this->declare_parameter("SERVO_CONFIG_TOPIC");
    this->declare_parameter("ACCEL_CONFIG_REQ_MCU_TOPIC");
    this->declare_parameter("ACCEL_CONFIG_TOPIC");
    this->declare_parameter("GYRO_CONFIG_REQ_MCU_TOPIC");
    this->declare_parameter("GYRO_CONFIG_TOPIC");
    this->declare_parameter("MAG_CONFIG_REQ_MCU_TOPIC");
    this->declare_parameter("MAG_CONFIG_TOPIC");

    // Subscribers
    this->declare_parameter("MASS_CONFIG_REQ_JETSON_TOPIC");
    this->declare_parameter("POT_CONFIG_REQ_JETSON_TOPIC");
    this->declare_parameter("SERVO_CONFIG_REQ_JETSON_TOPIC");
    this->declare_parameter("ACCEL_CONFIG_REQ_JETSON_TOPIC");
    this->declare_parameter("GYRO_CONFIG_REQ_JETSON_TOPIC");
    this->declare_parameter("MAG_CONFIG_REQ_JETSON_TOPIC");

    // Calibration parameters ======================
    // Mass sensor
    this->declare_parameter("mass_drill.offset");
    this->declare_parameter("mass_drill.scale");
    this->declare_parameter("mass_drill.alpha");
    this->declare_parameter("mass_drill.enabled_channels");

    this->declare_parameter("mass_container.offset");
    this->declare_parameter("mass_container.scale");
    this->declare_parameter("mass_container.alpha");
    this->declare_parameter("mass_container.enabled_channels");

    // Potentiometer
    this->declare_parameter("potentiometer.min_voltages");
    this->declare_parameter("potentiometer.max_voltages");
    this->declare_parameter("potentiometer.min_angles");
    this->declare_parameter("potentiometer.max_angles");
    this->declare_parameter("potentiometer.enabled_channels");

    // Servo
    this->declare_parameter("servo.min_duty");
    this->declare_parameter("servo.max_duty");
    this->declare_parameter("servo.min_angles");
    this->declare_parameter("servo.max_angles");

    // Accelerometer
    this->declare_parameter("accel.bias");
    this->declare_parameter("accel.transform");

    // Gyroscope
    this->declare_parameter("gyro.bias");

    // Magnetometer
    this->declare_parameter("mag.hard_iron");
    this->declare_parameter("mag.soft_iron");

    this->mass_config_req_pub = this->create_publisher<avionics_interfaces::msg::MassConfigRequestJetson>
        (get_param<std::string>("MASS_CONFIG_REQ_JETSON_TOPIC"), 10);
    this->pot_config_req_pub = this->create_publisher<avionics_interfaces::msg::PotConfigRequestJetson>
        (get_param<std::string>("POT_CONFIG_REQ_JETSON_TOPIC"), 10);
    this->servo_config_req_pub = this->create_publisher<avionics_interfaces::msg::ServoConfigRequestJetson>
        (get_param<std::string>("SERVO_CONFIG_REQ_JETSON_TOPIC"), 10);
    this->accel_config_req_pub = this->create_publisher<avionics_interfaces::msg::AccelConfigRequestJetson>
        (get_param<std::string>("ACCEL_CONFIG_REQ_JETSON_TOPIC"), 10);
    this->gyro_config_req_pub = this->create_publisher<avionics_interfaces::msg::GyroConfigRequestJetson>
        (get_param<std::string>("GYRO_CONFIG_REQ_JETSON_TOPIC"), 10);
    this->mag_config_req_pub = this->create_publisher<avionics_interfaces::msg::MagConfigRequestJetson>
        (get_param<std::string>("MAG_CONFIG_REQ_JETSON_TOPIC"), 10);

    this->mass_config_response_sub = this->create_subscription<avionics_interfaces::msg::MassConfigResponse>
        (get_param<std::string>("MASS_CONFIG_TOPIC"), 10, std::bind(&ConfigManager::massConfigResponseCallback, this, _1));
    this->mass_config_req_sub = this->create_subscription<avionics_interfaces::msg::MassConfigRequestMCU>
        (get_param<std::string>("MASS_CONFIG_REQ_MCU_TOPIC"), 10, std::bind(&ConfigManager::massConfigReqCallback, this, _1));
    this->pot_config_req_sub = this->create_subscription<avionics_interfaces::msg::PotConfigRequestMCU>
        (get_param<std::string>("POT_CONFIG_REQ_MCU_TOPIC"), 10, std::bind(&ConfigManager::potConfigReqCallback, this, _1));
    this->pot_config_response_sub = this->create_subscription<avionics_interfaces::msg::PotConfigResponse>
        (get_param<std::string>("POT_CONFIG_TOPIC"), 10, std::bind(&ConfigManager::potConfigResponseCallback, this, _1));
    this->servo_config_req_sub = this->create_subscription<avionics_interfaces::msg::ServoConfigRequestMCU>
        (get_param<std::string>("SERVO_CONFIG_REQ_MCU_TOPIC"), 10, std::bind(&ConfigManager::servoConfigReqCallback, this, _1));
    this->servo_config_response_sub = this->create_subscription<avionics_interfaces::msg::ServoConfigResponse>
        (get_param<std::string>("SERVO_CONFIG_TOPIC"), 10, std::bind(&ConfigManager::servoConfigResponseCallback, this, _1));
    this->accel_config_req_sub = this->create_subscription<avionics_interfaces::msg::AccelConfigRequestMCU>
        (get_param<std::string>("ACCEL_CONFIG_REQ_MCU_TOPIC"), 10, std::bind(&ConfigManager::accelConfigReqCallback, this, _1));
    this->accel_config_response_sub = this->create_subscription<avionics_interfaces::msg::AccelConfigResponse>
        (get_param<std::string>("ACCEL_CONFIG_TOPIC"), 10, std::bind(&ConfigManager::accelConfigResponseCallback, this, _1));
    this->gyro_config_req_sub = this->create_subscription<avionics_interfaces::msg::GyroConfigRequestMCU>
        (get_param<std::string>("GYRO_CONFIG_REQ_MCU_TOPIC"), 10, std::bind(&ConfigManager::gyroConfigReqCallback, this, _1));
    this->gyro_config_response_sub = this->create_subscription<avionics_interfaces::msg::GyroConfigResponse>
        (get_param<std::string>("GYRO_CONFIG_TOPIC"), 10, std::bind(&ConfigManager::gyroConfigResponseCallback, this, _1));
    this->mag_config_req_sub = this->create_subscription<avionics_interfaces::msg::MagConfigRequestMCU>
        (get_param<std::string>("MAG_CONFIG_REQ_MCU_TOPIC"), 10, std::bind(&ConfigManager::magConfigReqCallback, this, _1));
    this->mag_config_response_sub = this->create_subscription<avionics_interfaces::msg::MagConfigResponse>
        (get_param<std::string>("MAG_CONFIG_TOPIC"), 10, std::bind(&ConfigManager::magConfigResponseCallback, this, _1));

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

void ConfigManager::potConfigReqCallback(const avionics_interfaces::msg::PotConfigRequestMCU::SharedPtr msg) {
    auto msg_req = avionics_interfaces::msg::PotConfigRequestJetson();
    msg_req.destination_id = msg->id;
    msg_req.remote_command = false; // request from MCU (and not from Jetson) -> false
    msg_req.set_min_angles = msg->req_min_angles;
    msg_req.set_max_angles = msg->req_max_angles;
    msg_req.set_min_voltages = msg->req_min_voltages;
    msg_req.set_max_voltages = msg->req_max_voltages;
    msg_req.set_channels_status = msg->req_channels_status;

    std::string sensor = "potentiometer";

    std::vector<double> min_voltages_double = get_param<std::vector<double>>(sensor + ".min_voltages");
    std::vector<double> max_voltages_double = get_param<std::vector<double>>(sensor + ".max_voltages");
    std::vector<double> min_angles_double = get_param<std::vector<double>>(sensor + ".min_angles");
    std::vector<double> max_angles_double = get_param<std::vector<double>>(sensor + ".max_angles");
    
    for (uint8_t i = 0; i < 4; ++i) {
        msg_req.min_voltages[i] = min_voltages_double[i];
        msg_req.max_voltages[i] = max_voltages_double[i];
        msg_req.min_angles[i] = min_angles_double[i];
        msg_req.max_angles[i] = max_angles_double[i];
        msg_req.enabled_channels[i] = get_param<std::vector<bool>>(sensor + ".enabled_channels")[i];
    }
    
    pot_config_req_pub->publish(msg_req);
}


void ConfigManager::potConfigResponseCallback(const avionics_interfaces::msg::PotConfigResponse::SharedPtr msg) {

    std::string sensor = "potentiometer";

    RCLCPP_INFO(this->get_logger(), "Saving potentiometer calibration parameters into YAML file...");
    std::vector<double> min_angles_vector;
    std::vector<double> max_angles_vector;
    std::vector<double> min_voltages_vector;
    std::vector<double> max_voltages_vector;
    std::vector<bool> enabled_channels_vector;
    for (uint8_t i = 0; i < 4; i++) {
        min_angles_vector.push_back(static_cast<double>(msg->min_angles[i]));
        max_angles_vector.push_back(static_cast<double>(msg->max_angles[i]));
        min_voltages_vector.push_back(static_cast<double>(msg->min_voltages[i]));
        max_voltages_vector.push_back(static_cast<double>(msg->max_voltages[i]));
        enabled_channels_vector.push_back(msg->enabled_channels[i]);
    }

    // Set parameters into YAML file
    if (msg->set_min_angles)
        set_param_calib(sensor, "min_angles", min_angles_vector);
    if (msg->set_max_angles)
        set_param_calib(sensor, "max_angles", max_angles_vector);
    if (msg->set_min_voltages)
        set_param_calib(sensor, "min_voltages", min_voltages_vector);
    if (msg->set_max_voltages)
        set_param_calib(sensor, "max_voltages", max_voltages_vector);
    if (msg->set_channels_status)
        set_param_calib(sensor, "enabled_channels", enabled_channels_vector);
}

void ConfigManager::servoConfigReqCallback(const avionics_interfaces::msg::ServoConfigRequestMCU::SharedPtr msg) {
    auto msg_req = avionics_interfaces::msg::ServoConfigRequestJetson();
    msg_req.destination_id = msg->id;
    msg_req.remote_command = false; // request from MCU (and not from Jetson) -> false
    msg_req.set_min_angles = msg->req_min_angles;
    msg_req.set_max_angles = msg->req_max_angles;
    msg_req.set_min_duty = msg->req_min_duty;
    msg_req.set_max_duty = msg->req_max_duty;

    std::string sensor = "servo";

    std::vector<double> min_duty_double = get_param<std::vector<double>>(sensor + ".min_duty");
    std::vector<double> max_duty_double = get_param<std::vector<double>>(sensor + ".max_duty");
    std::vector<double> min_angles_double = get_param<std::vector<double>>(sensor + ".min_angles");
    std::vector<double> max_angles_double = get_param<std::vector<double>>(sensor + ".max_angles");
    
    for (uint8_t i = 0; i < 4; ++i) {
        msg_req.min_duty[i] = min_duty_double[i];
        msg_req.max_duty[i] = max_duty_double[i];
        msg_req.min_angles[i] = min_angles_double[i];
        msg_req.max_angles[i] = max_angles_double[i];
    }
    
    servo_config_req_pub->publish(msg_req);
}

void ConfigManager::servoConfigResponseCallback(const avionics_interfaces::msg::ServoConfigResponse::SharedPtr msg) {
    std::string sensor = "servo";

    RCLCPP_INFO(this->get_logger(), "Saving servo calibration parameters into YAML file...");
    std::vector<double> min_angles_vector;
    std::vector<double> max_angles_vector;
    std::vector<double> min_duty_vector;
    std::vector<double> max_duty_vector;
    for (uint8_t i = 0; i < 4; i++) {
        min_angles_vector.push_back(static_cast<double>(msg->min_angles[i]));
        max_angles_vector.push_back(static_cast<double>(msg->max_angles[i]));
        min_duty_vector.push_back(static_cast<double>(msg->min_duty[i]));
        max_duty_vector.push_back(static_cast<double>(msg->max_duty[i]));
    }

    // Set parameters into YAML file
    if (msg->set_min_angles)
        set_param_calib(sensor, "min_angles", min_angles_vector);
    if (msg->set_max_angles)
        set_param_calib(sensor, "max_angles", max_angles_vector);
    if (msg->set_min_duty)
        set_param_calib(sensor, "min_duty", min_duty_vector);
    if (msg->set_max_duty)
        set_param_calib(sensor, "max_duty", max_duty_vector);
}

void ConfigManager::accelConfigReqCallback(const avionics_interfaces::msg::AccelConfigRequestMCU::SharedPtr msg) {
    auto msg_req = avionics_interfaces::msg::AccelConfigRequestJetson();
    msg_req.destination_id = msg->id;
    msg_req.remote_command = false; // request from MCU (and not from Jetson) -> false
    msg_req.set_bias = msg->req_bias;
    msg_req.set_transform = msg->req_transform;

    std::string sensor = "accel";

    std::vector<double> bias_double = get_param<std::vector<double>>(sensor + ".bias");
    std::vector<double> transform_double = get_param<std::vector<double>>(sensor + ".transform");
    
    for (uint8_t i = 0; i < 3; ++i) 
        msg_req.bias[i] = bias_double[i];

    for (uint8_t i = 0; i < 9; ++i)
        msg_req.transform[i] = transform_double[i];
    
    accel_config_req_pub->publish(msg_req);
}

void ConfigManager::accelConfigResponseCallback(const avionics_interfaces::msg::AccelConfigResponse::SharedPtr msg) {
    std::string sensor = "accel";

    RCLCPP_INFO(this->get_logger(), "Saving accelerometer calibration parameters into YAML file...");
    std::vector<double> bias_vector;
    std::vector<double> transform_vector;
    for (uint8_t i = 0; i < 3; i++) 
        bias_vector.push_back(static_cast<double>(msg->bias[i]));

    for (uint8_t i = 0; i < 9; i++) 
        transform_vector.push_back(static_cast<double>(msg->transform[i]));

    // Set parameters into YAML file
    if (msg->set_bias)
        set_param_calib(sensor, "bias", bias_vector);
    if (msg->set_transform)
        set_param_calib(sensor, "transform", transform_vector);
}

void ConfigManager::gyroConfigReqCallback(const avionics_interfaces::msg::GyroConfigRequestMCU::SharedPtr msg) {
    auto msg_req = avionics_interfaces::msg::GyroConfigRequestJetson();
    msg_req.destination_id = msg->id;
    msg_req.remote_command = false; // request from MCU (and not from Jetson) -> false
    msg_req.set_bias = msg->req_bias;

    std::string sensor = "gyro";

    std::vector<double> bias_double = get_param<std::vector<double>>(sensor + ".bias");
    
    for (uint8_t i = 0; i < 3; ++i) 
        msg_req.bias[i] = bias_double[i];
    
    gyro_config_req_pub->publish(msg_req);
}

void ConfigManager::gyroConfigResponseCallback(const avionics_interfaces::msg::GyroConfigResponse::SharedPtr msg) {
    std::string sensor = "gyro";

    RCLCPP_INFO(this->get_logger(), "Saving gyroscope calibration parameters into YAML file...");
    std::vector<double> bias_vector;
    std::vector<double> transform_vector;
    for (uint8_t i = 0; i < 3; i++) 
        bias_vector.push_back(static_cast<double>(msg->bias[i]));

    // Set parameters into YAML file
    if (msg->set_bias)
        set_param_calib(sensor, "bias", bias_vector);
}

void ConfigManager::magConfigReqCallback(const avionics_interfaces::msg::MagConfigRequestMCU::SharedPtr msg) {
    auto msg_req = avionics_interfaces::msg::MagConfigRequestJetson();
    msg_req.destination_id = msg->id;
    msg_req.remote_command = false; // request from MCU (and not from Jetson) -> false
    msg_req.set_hard_iron = msg->req_hard_iron;
    msg_req.set_soft_iron = msg->req_soft_iron;

    std::string sensor = "mag";

    std::vector<double> hard_iron_double = get_param<std::vector<double>>(sensor + ".hard_iron");
    std::vector<double> soft_iron_double = get_param<std::vector<double>>(sensor + ".soft_iron");
    
    for (uint8_t i = 0; i < 3; ++i) 
        msg_req.hard_iron[i] = hard_iron_double[i];

    for (uint8_t i = 0; i < 9; ++i)
        msg_req.soft_iron[i] = soft_iron_double[i];
    
    mag_config_req_pub->publish(msg_req);
}

void ConfigManager::magConfigResponseCallback(const avionics_interfaces::msg::MagConfigResponse::SharedPtr msg) {
    std::string sensor = "mag";

    RCLCPP_INFO(this->get_logger(), "Saving magnetometer calibration parameters into YAML file...");
    std::vector<double> hard_iron_vector;
    std::vector<double> soft_iron_vector;
    for (uint8_t i = 0; i < 3; i++) 
        hard_iron_vector.push_back(static_cast<double>(msg->hard_iron[i]));

    for (uint8_t i = 0; i < 9; i++) 
        soft_iron_vector.push_back(static_cast<double>(msg->soft_iron[i]));

    // Set parameters into YAML file
    if (msg->set_hard_iron)
        set_param_calib(sensor, "hard_iron", hard_iron_vector);
    if (msg->set_soft_iron)
        set_param_calib(sensor, "soft_iron", soft_iron_vector);
}