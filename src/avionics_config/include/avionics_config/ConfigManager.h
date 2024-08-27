/*
 * ConfigManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include "custom_msg/msg/mass_config_request_mcu.hpp"
#include "custom_msg/msg/mass_config_response.hpp"

#include "custom_msg/msg/pot_config_request_mcu.hpp"
#include "custom_msg/msg/pot_config_response.hpp"

#include "custom_msg/msg/servo_config_request_mcu.hpp"
#include "custom_msg/msg/servo_config_response.hpp"

#include "custom_msg/msg/accel_config_request_mcu.hpp"
#include "custom_msg/msg/accel_config_response.hpp"

#include "custom_msg/msg/gyro_config_request_mcu.hpp"
#include "custom_msg/msg/gyro_config_response.hpp"

#include "custom_msg/msg/mag_config_request_mcu.hpp"
#include "custom_msg/msg/mag_config_response.hpp"

#include "custom_msg/msg/mass_config_request_jetson.hpp"
#include "custom_msg/msg/pot_config_request_jetson.hpp"
#include "custom_msg/msg/servo_config_request_jetson.hpp"
#include "custom_msg/msg/accel_config_request_jetson.hpp"
#include "custom_msg/msg/gyro_config_request_jetson.hpp"
#include "custom_msg/msg/mag_config_request_jetson.hpp"

class ConfigManager : public rclcpp::Node {
public:
    ConfigManager();
    ~ConfigManager();

    // For writing back to YAML file.
    template <typename T>
    void update_yaml_file(const std::string& yaml_file_path, const std::string sensor, const std::string& parameter_name, T value) {
        // Load the YAML file
        YAML::Node yaml_node = YAML::LoadFile(yaml_file_path);

        // Function to add a small value to a double if it's close to a whole number
        // This is to avoid mixed types errors in YAML which crashes the node
        auto addSmallValueIfNeeded = [](auto& val) {
            using ValueType = std::decay_t<decltype(val)>;
            if (std::abs(val - std::round(val)) < static_cast<ValueType>(1e-9)) {
                val += static_cast<ValueType>(0.000000001);
            }
        };

        // Apply the small value adjustment if needed to each element in the value parameter
        if constexpr (std::is_same<T, std::vector<std::vector<double>>>::value) {
            for (std::vector<double>& vec: value)
                for (double& val : vec)
                    addSmallValueIfNeeded(val);
        }
        else if constexpr (std::is_same<T, std::vector<double>>::value) {
            for (double& val : value) {
                addSmallValueIfNeeded(val);
            }
        } else if constexpr (std::is_same<T, double>::value) {
            addSmallValueIfNeeded(value);
        } else if constexpr (std::is_same<T, float>::value) {
            addSmallValueIfNeeded(value);
        }

        // Set the modified value parameter in the YAML node
        yaml_node["/**"]["ros__parameters"][sensor][parameter_name] = value;

        // Write the updated content back to the file
        std::ofstream yaml_file(yaml_file_path);
        if (yaml_file) {
            yaml_file << yaml_node;
            yaml_file.close();
        } else {
            std::cerr << "Error opening YAML file for writing." << std::endl;
            // Handle the error appropriately
        }
    }



    template <typename T>
    T get_param(const std::string& parameter_name) {
        T value;
        if (this->get_parameter(parameter_name, value)) {
            return value;
        } else {
            RCLCPP_WARN(this->get_logger(), "Parameter [%s] not found, using default value.", parameter_name.c_str());
            return T();
        }
    }

    template <typename T>
    bool set_param_calib(const std::string& sensor, const std::string& parameter_name, const T& value)
    {
        rclcpp::Parameter parameter(sensor + "." + parameter_name, value);
        auto result = this->set_parameter(parameter); // set parameter in parameter server

        if (result.successful) {
            std::string yaml_file_path = "src/avionics_config/config/calibration_params.yaml";
            update_yaml_file(yaml_file_path, sensor, parameter_name, value);
            return true;
        } else {
            return false;
        }
    }

private:
    rclcpp::Subscription<custom_msg::msg::MassConfigResponse>::SharedPtr mass_config_response_sub;
    rclcpp::Subscription<custom_msg::msg::MassConfigRequestMCU>::SharedPtr mass_config_req_sub;

    rclcpp::Subscription<custom_msg::msg::PotConfigResponse>::SharedPtr pot_config_response_sub;
    rclcpp::Subscription<custom_msg::msg::PotConfigRequestMCU>::SharedPtr pot_config_req_sub;

    rclcpp::Subscription<custom_msg::msg::ServoConfigResponse>::SharedPtr servo_config_response_sub;
    rclcpp::Subscription<custom_msg::msg::ServoConfigRequestMCU>::SharedPtr servo_config_req_sub;

    rclcpp::Subscription<custom_msg::msg::AccelConfigResponse>::SharedPtr accel_config_response_sub;
    rclcpp::Subscription<custom_msg::msg::AccelConfigRequestMCU>::SharedPtr accel_config_req_sub;

    rclcpp::Subscription<custom_msg::msg::GyroConfigResponse>::SharedPtr gyro_config_response_sub;
    rclcpp::Subscription<custom_msg::msg::GyroConfigRequestMCU>::SharedPtr gyro_config_req_sub;

    rclcpp::Subscription<custom_msg::msg::MagConfigResponse>::SharedPtr mag_config_response_sub;
    rclcpp::Subscription<custom_msg::msg::MagConfigRequestMCU>::SharedPtr mag_config_req_sub;

    rclcpp::Publisher<custom_msg::msg::MassConfigRequestJetson>::SharedPtr mass_config_req_pub;
    rclcpp::Publisher<custom_msg::msg::PotConfigRequestJetson>::SharedPtr pot_config_req_pub;
    rclcpp::Publisher<custom_msg::msg::ServoConfigRequestJetson>::SharedPtr servo_config_req_pub;
    rclcpp::Publisher<custom_msg::msg::AccelConfigRequestJetson>::SharedPtr accel_config_req_pub;
    rclcpp::Publisher<custom_msg::msg::GyroConfigRequestJetson>::SharedPtr gyro_config_req_pub;
    rclcpp::Publisher<custom_msg::msg::MagConfigRequestJetson>::SharedPtr mag_config_req_pub;

    void massConfigReqCallback(const custom_msg::msg::MassConfigRequestMCU::SharedPtr msg);
    void massConfigResponseCallback(const custom_msg::msg::MassConfigResponse::SharedPtr msg);

    void potConfigReqCallback(const custom_msg::msg::PotConfigRequestMCU::SharedPtr msg);
    void potConfigResponseCallback(const custom_msg::msg::PotConfigResponse::SharedPtr msg);

    void servoConfigReqCallback(const custom_msg::msg::ServoConfigRequestMCU::SharedPtr msg);
    void servoConfigResponseCallback(const custom_msg::msg::ServoConfigResponse::SharedPtr msg);

    void accelConfigReqCallback(const custom_msg::msg::AccelConfigRequestMCU::SharedPtr msg);
    void accelConfigResponseCallback(const custom_msg::msg::AccelConfigResponse::SharedPtr msg);

    void gyroConfigReqCallback(const custom_msg::msg::GyroConfigRequestMCU::SharedPtr msg);
    void gyroConfigResponseCallback(const custom_msg::msg::GyroConfigResponse::SharedPtr msg);

    void magConfigReqCallback(const custom_msg::msg::MagConfigRequestMCU::SharedPtr msg);
    void magConfigResponseCallback(const custom_msg::msg::MagConfigResponse::SharedPtr msg);
};

#endif /* CONFIG_MANAGER_H */