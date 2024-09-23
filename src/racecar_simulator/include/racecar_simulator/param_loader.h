#pragma once

#include <rclcpp/rclcpp.hpp>
#include "racecar_simulator/car_params.h"

struct Params
{
    std::string drive_topic0;
    std::string state_topic0;
    std::string scan_topic0;
    std::string drive_topic1;
    std::string state_topic1;
    std::string scan_topic1;
    double simulator_frequency;
    double pub_frequency;
    int scan_beams;
    double scan_field_of_view;
    double scan_std_dev;
    double map_free_threshold;
    bool detect_car_mode;
    bool state_noise_mode;
    bool scan_noise_mode;
    std::string pgm_file_path;
    std::string yaml_file_path;

    CarParams p0;
    CarParams p1;
};

Params load_parameters(rclcpp::Node * node);


