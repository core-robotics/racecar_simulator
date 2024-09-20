#include "param_loader.h"
#include "car_params.h"

Params load_parameters(rclcpp::Node * node)
{
    Params params;
    node->declare_parameter("simulator_frequency", 1000.0);
    node->declare_parameter("pub_frequency", 40.0);
    node->declare_parameter("scan_beams", 1080);
    node->declare_parameter("scan_field_of_view", 2.0 * M_PI);
    node->declare_parameter("scan_std_dev", 0.01);
    node->declare_parameter("map_free_threshold", 0.2);
    node->declare_parameter("detect_car_mode", false);
    node->declare_parameter("state_noise_mode", false);
    node->declare_parameter("scan_noise_mode", false);
    node->declare_parameter<std::string>("pgm_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.pgm");
    node->declare_parameter<std::string>("yaml_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.yaml");

    node->get_parameter("simulator_frequency", params.simulator_frequency);
    node->get_parameter("pub_frequency", params.pub_frequency);
    node->get_parameter("scan_beams", params.scan_beams);
    node->get_parameter("scan_field_of_view", params.scan_field_of_view);
    node->get_parameter("scan_std_dev", params.scan_std_dev);
    node->get_parameter("map_free_threshold", params.map_free_threshold);
    node->get_parameter("detect_car_mode", params.detect_car_mode);
    node->get_parameter("state_noise_mode", params.state_noise_mode);
    node->get_parameter("scan_noise_mode", params.scan_noise_mode);
    node->get_parameter("pgm_file_path", params.pgm_file_path);
    node->get_parameter("yaml_file_path", params.yaml_file_path);

    CarParams p0,p1;
    // Car0 parameters
    node->declare_parameter("vehicle_model0",1);
    node->declare_parameter("drive_topic0", "ackermann_cmd0");
    node->declare_parameter("state_topic0", "state0");
    node->declare_parameter("scan_topic0", "scan0");
    node->declare_parameter("friction_coeff0", p0.friction_coeff);
    node->declare_parameter("mass0", p0.mass);
    node->declare_parameter("l_r0", p0.l_r);
    node->declare_parameter("l_f0", p0.l_f);
    node->declare_parameter("I_z0", p0.I_z);
    node->declare_parameter("cs_f0", p0.cs_f);
    node->declare_parameter("cs_r0", p0.cs_r);
    node->declare_parameter("h_cg0", p0.h_cg);
    node->declare_parameter("B_f0", p0.Bf);
    node->declare_parameter("C_f0", p0.Cf);
    node->declare_parameter("D_f0", p0.Df);
    node->declare_parameter("B_r0", p0.Br);
    node->declare_parameter("C_r0", p0.Cr);
    node->declare_parameter("D_r0", p0.Dr);
    node->declare_parameter("I_z0", p0.I_z);
    



    
};