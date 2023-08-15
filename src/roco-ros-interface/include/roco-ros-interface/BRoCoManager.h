/*
 * BRoCoManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef BROCO_MANAGER
#define BROCO_MANAGER

#include "BRoCoPublisher.h"
#include "BRoCoSubscriber.h"

#include "BRoCo/CanSocketDriver.h"


#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

class BRoCoManager : public rclcpp::Node {
public:
  BRoCoManager();
  ~BRoCoManager();

  template <typename T> 
  T get_param(const std::string& parameter_name);

  // For some reason these functions have to be defined in the header file
  // We write custom functions for yaml files to be able to write to the same
  // Config file from multiple nodes
  template <typename T>
  void update_yaml_file(const std::string& yaml_file_path, const std::string& parameter_name, const T& value)
  {
      // Load the YAML file
      YAML::Node yaml_node = YAML::LoadFile(yaml_file_path);

      // Find and update the parameter value
      if (yaml_node["/**"]["ros__parameters"][parameter_name]) {
        RCLCPP_INFO(this->get_logger(), "Parameter [%s] set to: %d", parameter_name.c_str(), value);
          yaml_node["/**"]["ros__parameters"][parameter_name] = value;
      }

      // Write the updated content back to the file
      std::ofstream yaml_file(yaml_file_path);
      yaml_file << yaml_node;
      yaml_file.close();
  }

  template <typename T>
  bool set_param(const std::string& parameter_name, const T& value)
  {
      rclcpp::Parameter parameter(parameter_name, value);
      auto result = this->set_parameter(parameter);

      if (result.successful) {
          std::string yaml_file_path = "src/roco-ros-interface/config/node_ids_params.yaml";
          update_yaml_file(yaml_file_path, parameter_name, value);
          return true;
      } else {
          return false;
      }
  }

private:
    CanSocketDriver* driver;
    CANBus* bus;
    BRoCoPublisher* pub;
    BRoCoSubscriber* sub;
};

#endif /* BROCO_MANAGER */