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

  // These functions have to be written in the header for the compiler
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
  T get_param(const std::string& parameter_name)
  {
      T value;
      if (this->get_parameter(parameter_name, value)) {
          return value;
      } else {
          RCLCPP_WARN(this->get_logger(), "Parameter [%s] not found, using default value.", parameter_name.c_str());
          return T();
      }
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