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

#include "avionics_interfaces/msg/mass_config_request_jetson.hpp"
#include "avionics_interfaces/msg/mass_config_response.hpp"
#include "avionics_interfaces/msg/mass_config_request_mcu.hpp"

class ConfigManager : public rclcpp::Node {
public:
    ConfigManager();
    ~ConfigManager();

    // These functions have to be written in the header for the compiler
    template <typename T>
    void update_yaml_file(const std::string& yaml_file_path, const std::string sensor, const std::string& parameter_name, const T& value) {
        // Load the YAML file
        YAML::Node yaml_node = YAML::LoadFile(yaml_file_path);

        // Find and update the parameter value
        if (yaml_node["/**"]["ros__parameters"][sensor][parameter_name]) {
        yaml_node["/**"]["ros__parameters"][sensor][parameter_name] = value;
        }

        // Write the updated content back to the file
        std::ofstream yaml_file(yaml_file_path);
        yaml_file << yaml_node;
        yaml_file.close();
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
        auto result = this->set_parameter(parameter);

        if (result.successful) {
            std::string yaml_file_path = "src/avionics_config/config/calibration_params.yaml";
            update_yaml_file(yaml_file_path, sensor, parameter_name, value);
            return true;
        } else {
            return false;
        }
    }

private:
    rclcpp::Subscription<avionics_interfaces::msg::MassConfigResponse>::SharedPtr mass_config_response_sub;
    rclcpp::Subscription<avionics_interfaces::msg::MassConfigRequestMCU>::SharedPtr mass_config_req_sub;

    rclcpp::Publisher<avionics_interfaces::msg::MassConfigRequestJetson>::SharedPtr mass_config_req_pub;

    void massConfigReqCallback(const avionics_interfaces::msg::MassConfigRequestMCU::SharedPtr msg);
    void massConfigResponseCallback(const avionics_interfaces::msg::MassConfigResponse::SharedPtr msg);
};

#endif /* CONFIG_MANAGER_H */