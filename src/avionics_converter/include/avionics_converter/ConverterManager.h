/*
 * ConverterManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef CONVERTER_MANAGER_H
#define CONVERTER_MANAGER_H

#include "ConverterPublisher.h"

class ConverterManager : public rclcpp::Node {
public:
    ConverterManager();
    ~ConverterManager();

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
    ConverterPublisher* pub;
};

#endif /* CONVERTER_MANAGER_H */