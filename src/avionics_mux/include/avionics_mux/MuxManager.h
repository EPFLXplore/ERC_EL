/*
 * MuxManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef MUX_MANAGER_H
#define MUX_MANAGER_H

#define NOBUS (2)

#include "avionics_interfaces/msg/four_in_one.hpp"
#include "avionics_interfaces/msg/npk.hpp"
#include "avionics_interfaces/msg/voltage.hpp"
#include "avionics_interfaces/msg/mass_array.hpp"
#include "avionics_interfaces/msg/imu.hpp"
#include "avionics_interfaces/msg/mag.hpp"
#include "avionics_interfaces/msg/angle_array.hpp"
#include "avionics_interfaces/msg/spectro_response.hpp"
#include "avionics_interfaces/msg/laser_response.hpp"
#include "avionics_interfaces/msg/servo_response.hpp"
#include "avionics_interfaces/msg/led_response.hpp"
#include "avionics_interfaces/msg/node_state_array.hpp"

#include "avionics_interfaces/msg/laser_request.hpp"
#include "avionics_interfaces/msg/led_request.hpp"
#include "avionics_interfaces/msg/servo_request.hpp"
#include "avionics_interfaces/msg/spectro_request.hpp"

#include "avionics_interfaces/msg/mass_config_request_mcu.hpp"
#include "avionics_interfaces/msg/mass_config_response.hpp"

#include "avionics_interfaces/msg/pot_config_request_mcu.hpp"
#include "avionics_interfaces/msg/pot_config_response.hpp"

#include "avionics_interfaces/msg/servo_config_request_mcu.hpp"
#include "avionics_interfaces/msg/servo_config_response.hpp"

#include "avionics_interfaces/msg/accel_config_request_mcu.hpp"
#include "avionics_interfaces/msg/accel_config_response.hpp"

#include "avionics_interfaces/msg/gyro_config_request_mcu.hpp"
#include "avionics_interfaces/msg/gyro_config_response.hpp"

#include "avionics_interfaces/msg/mag_config_request_mcu.hpp"
#include "avionics_interfaces/msg/mag_config_response.hpp"

#include "avionics_interfaces/msg/mass_config_request_jetson.hpp"
#include "avionics_interfaces/msg/pot_config_request_jetson.hpp"
#include "avionics_interfaces/msg/servo_config_request_jetson.hpp"
#include "avionics_interfaces/msg/accel_config_request_jetson.hpp"
#include "avionics_interfaces/msg/gyro_config_request_jetson.hpp"
#include "avionics_interfaces/msg/mag_config_request_jetson.hpp"

#include "avionics_interfaces/msg/mass_calib_offset.hpp"
#include "avionics_interfaces/msg/mass_calib_scale.hpp"
#include "avionics_interfaces/msg/imu_calib.hpp"

#include "MuxPublisher.h"
#include "MuxSubscriber.h"

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
    uint16_t get_node_id(std::string node_name);

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
    MuxPublisher<avionics_interfaces::msg::Mag>* mag_mux;
    MuxPublisher<avionics_interfaces::msg::AngleArray>* potentiometer_mux;
    MuxPublisher<avionics_interfaces::msg::SpectroResponse>* spectro_response_mux;
    MuxPublisher<avionics_interfaces::msg::LaserResponse>* laser_response_mux;
    MuxPublisher<avionics_interfaces::msg::ServoResponse>* servo_response_mux;
    MuxPublisher<avionics_interfaces::msg::LEDResponse>* led_response_mux;

    MuxPublisher<avionics_interfaces::msg::MassConfigRequestMCU>* mass_config_req_mcu_mux;
    MuxPublisher<avionics_interfaces::msg::MassConfigResponse>* mass_config_response_mux;

    MuxPublisher<avionics_interfaces::msg::PotConfigRequestMCU>* pot_config_req_mcu_mux;
    MuxPublisher<avionics_interfaces::msg::PotConfigResponse>* pot_config_response_mux;

    MuxPublisher<avionics_interfaces::msg::ServoConfigRequestMCU>* servo_config_req_mcu_mux;
    MuxPublisher<avionics_interfaces::msg::ServoConfigResponse>* servo_config_response_mux;

    MuxPublisher<avionics_interfaces::msg::AccelConfigRequestMCU>* accel_config_req_mcu_mux;
    MuxPublisher<avionics_interfaces::msg::AccelConfigResponse>* accel_config_response_mux;

    MuxPublisher<avionics_interfaces::msg::GyroConfigRequestMCU>* gyro_config_req_mcu_mux;
    MuxPublisher<avionics_interfaces::msg::GyroConfigResponse>* gyro_config_response_mux;

    MuxPublisher<avionics_interfaces::msg::MagConfigRequestMCU>* mag_config_req_mcu_mux;
    MuxPublisher<avionics_interfaces::msg::MagConfigResponse>* mag_config_response_mux;

    MuxSubscriber<avionics_interfaces::msg::LaserRequest>* laser_req_mux;
    MuxSubscriber<avionics_interfaces::msg::LEDRequest>* led_req_mux;
    MuxSubscriber<avionics_interfaces::msg::ServoRequest>* servo_req_mux;
    MuxSubscriber<avionics_interfaces::msg::SpectroRequest>* spectro_req_mux;

    MuxSubscriber<avionics_interfaces::msg::MassConfigRequestJetson>* mass_config_req_jetson_mux;
    MuxSubscriber<avionics_interfaces::msg::PotConfigRequestJetson>* pot_config_req_jetson_mux;
    MuxSubscriber<avionics_interfaces::msg::ServoConfigRequestJetson>* servo_config_req_jetson_mux;
    MuxSubscriber<avionics_interfaces::msg::AccelConfigRequestJetson>* accel_config_req_jetson_mux;
    MuxSubscriber<avionics_interfaces::msg::GyroConfigRequestJetson>* gyro_config_req_jetson_mux;
    MuxSubscriber<avionics_interfaces::msg::MagConfigRequestJetson>* mag_config_req_jetson_mux;

    MuxSubscriber<avionics_interfaces::msg::MassCalibOffset>* mass_container_calib_offset_mux;
    MuxSubscriber<avionics_interfaces::msg::MassCalibOffset>* mass_drill_calib_offset_mux;
    MuxSubscriber<avionics_interfaces::msg::MassCalibScale>* mass_container_calib_scale_mux;
    MuxSubscriber<avionics_interfaces::msg::MassCalibScale>* mass_drill_calib_scale_mux;
    MuxSubscriber<avionics_interfaces::msg::ImuCalib>* imu_calib_mux;
};

#endif /* MUX_MANAGER_H */