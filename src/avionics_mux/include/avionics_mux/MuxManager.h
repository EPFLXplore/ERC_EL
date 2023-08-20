/*
 * MuxManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef MUX_MANAGER_H
#define MUX_MANAGER_H

#include "avionics_interfaces/msg/four_in_one.hpp"
#include "avionics_interfaces/msg/npk.hpp"
#include "avionics_interfaces/msg/voltage.hpp"
#include "avionics_interfaces/msg/mass_array.hpp"
#include "avionics_interfaces/msg/imu.hpp"
#include "avionics_interfaces/msg/angle_array.hpp"
#include "avionics_interfaces/msg/spectro_response.hpp"
#include "avionics_interfaces/msg/laser_response.hpp"
#include "avionics_interfaces/msg/servo_response.hpp"
#include "avionics_interfaces/msg/led_response.hpp"
#include "avionics_interfaces/msg/node_state_array.hpp"

#include "MuxPublisher.h"

class MuxManager : public rclcpp::Node {
public:
    MuxManager();
    ~MuxManager();

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

    std::vector<bool> get_bus0_state() const;
    std::vector<bool> get_bus1_state() const;
    uint32_t get_max_number_nodes() const;

private:
    rclcpp::Subscription<avionics_interfaces::msg::NodeStateArray>::SharedPtr bus0_node_state_sub;
    rclcpp::Subscription<avionics_interfaces::msg::NodeStateArray>::SharedPtr bus1_node_state_sub;

    void bus0StateCallback(const avionics_interfaces::msg::NodeStateArray::SharedPtr msg);
    void bus1StateCallback(const avionics_interfaces::msg::NodeStateArray::SharedPtr msg);

    std::string bus0 = "";
    std::string bus1 = "";

    std::vector<bool> bus0_state;
    std::vector<bool> bus1_state;

    uint32_t max_number_nodes = 16;

    MuxPublisher<avionics_interfaces::msg::FourInOne>* four_in_one_mux;
    MuxPublisher<avionics_interfaces::msg::NPK>* npk_mux;
    MuxPublisher<avionics_interfaces::msg::Voltage>* voltage_mux;
    MuxPublisher<avionics_interfaces::msg::MassArray>* drill_mass_mux;
    MuxPublisher<avionics_interfaces::msg::MassArray>* container_mass_mux;
    MuxPublisher<avionics_interfaces::msg::Imu>* imu_mux;
    MuxPublisher<avionics_interfaces::msg::AngleArray>* potentiometer_mux;
    MuxPublisher<avionics_interfaces::msg::SpectroResponse>* spectro_response_mux;
    MuxPublisher<avionics_interfaces::msg::LaserResponse>* laser_response_mux;
    MuxPublisher<avionics_interfaces::msg::ServoResponse>* servo_response_mux;
    MuxPublisher<avionics_interfaces::msg::LEDResponse>* led_response_mux;
};

#endif /* MUX_MANAGER_H */