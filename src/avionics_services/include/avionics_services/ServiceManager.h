/*
 * ServiceManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef SERVICE_MANAGER_H
#define SERVICE_MANAGER_H

#include "avionics_interfaces/msg/laser_request.hpp"
#include "avionics_interfaces/msg/led_request.hpp"
#include "avionics_interfaces/msg/servo_request.hpp"
#include "avionics_interfaces/msg/spectro_request.hpp"

#include "avionics_interfaces/srv/rotate_servo.hpp"

class ServiceManager : public rclcpp::Node {
public:
    ServiceManager();
    ~ServiceManager();

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

private:
    uint32_t max_number_nodes = 16;
};

#endif /* SERVICE_MANAGER_H */