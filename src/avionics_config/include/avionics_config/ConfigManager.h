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
                val += static_cast<ValueType>(1e-9);
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
    rclcpp::Subscription<avionics_interfaces::msg::MassConfigResponse>::SharedPtr mass_config_response_sub;
    rclcpp::Subscription<avionics_interfaces::msg::MassConfigRequestMCU>::SharedPtr mass_config_req_sub;

    rclcpp::Publisher<avionics_interfaces::msg::MassConfigRequestJetson>::SharedPtr mass_config_req_pub;

    void massConfigReqCallback(const avionics_interfaces::msg::MassConfigRequestMCU::SharedPtr msg);
    void massConfigResponseCallback(const avionics_interfaces::msg::MassConfigResponse::SharedPtr msg);
};

#endif /* CONFIG_MANAGER_H */