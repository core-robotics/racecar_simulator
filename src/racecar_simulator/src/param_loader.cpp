#include "racecar_simulator/param_loader.h"
#include "racecar_simulator/car_params.h"

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
    node->declare_parameter("mass0", 3.5);
    node->declare_parameter("l_r0", 0.17145);
    node->declare_parameter("l_f0", 0.17145);
    node->declare_parameter("I_z0", 0.04712);
    node->declare_parameter("h_cg0", 0.1);
    node->declare_parameter("cs_f0", 1.0);
    node->declare_parameter("cs_r0", 1.0);
    node->declare_parameter("B_f0", 1.5);
    node->declare_parameter("C_f0", 1.5);
    node->declare_parameter("D_f0", 30.0);
    node->declare_parameter("B_r0", 1.5);
    node->declare_parameter("C_r0", 1.5);
    node->declare_parameter("D_r0", 30.0);
    node->declare_parameter("steer_max0", 4.0);
    node->declare_parameter("steer_vel_max0", 4.0);
    node->declare_parameter("speed_max0", 10.0);
    node->declare_parameter("accel_max0", 40.0);
    node->declare_parameter("decel_max0", 40.0);
    node->declare_parameter("jerk_max0", 100.0);
   

    node->get_parameter("vehicle_model0", params.p0.model_type);
    node->get_parameter("drive_topic0", params.drive_topic0);
    node->get_parameter("state_topic0", params.state_topic0);
    node->get_parameter("scan_topic0", params.scan_topic0);
    node->get_parameter("mass0", params.p0.mass);
    node->get_parameter("l_r0", params.p0.l_r);
    node->get_parameter("l_f0", params.p0.l_f);
    node->get_parameter("I_z0", params.p0.I_z);
    node->get_parameter("h_cg0", params.p0.h_cg);
    node->get_parameter("cs_f0", params.p0.cs_f);
    node->get_parameter("cs_r0", params.p0.cs_r);
    node->get_parameter("B_f0", params.p0.Bf);
    node->get_parameter("C_f0", params.p0.Cf);
    node->get_parameter("D_f0", params.p0.Df);
    node->get_parameter("B_r0", params.p0.Br);
    node->get_parameter("C_r0", params.p0.Cr);
    node->get_parameter("D_r0", params.p0.Dr);
    node->get_parameter("steer_max0", params.p0.steer_max);
    node->get_parameter("steer_vel_max0", params.p0.steer_vel_max);
    node->get_parameter("speed_max0", params.p0.speed_max);
    node->get_parameter("accel_max0", params.p0.accel_max);
    node->get_parameter("decel_max0", params.p0.decel_max);
    node->get_parameter("jerk_max0", params.p0.jerk_max);

    // Car1 parameters
    node->declare_parameter("vehicle_model1",1);
    node->declare_parameter("drive_topic1", "ackermann_cmd1");
    node->declare_parameter("state_topic1", "state1");
    node->declare_parameter("scan_topic1", "scan1");
    node->declare_parameter("mass1", 3.5);
    node->declare_parameter("l_r1", 0.17145);
    node->declare_parameter("l_f1", 0.17145);
    node->declare_parameter("I_z1", 0.04712);
    node->declare_parameter("h_cg1", 0.1);
    node->declare_parameter("cs_f1", 1.0);
    node->declare_parameter("cs_r1", 1.0);
    node->declare_parameter("B_f1", 1.5);
    node->declare_parameter("C_f1", 1.5);
    node->declare_parameter("D_f1", 30.0);
    node->declare_parameter("B_r1", 1.5);
    node->declare_parameter("C_r1", 1.5);
    node->declare_parameter("D_r1", 30.0);
    node->declare_parameter("steer_max1", 0.4);
    node->declare_parameter("steer_vel_max1", 0.041);
    node->declare_parameter("speed_max1", 10.0);
    node->declare_parameter("accel_max1", 4.0);
    node->declare_parameter("decel_max1", 4.0);
    node->declare_parameter("jerk_max1", 1.0);

    node->get_parameter("vehicle_model1", params.p1.model_type);
    node->get_parameter("drive_topic1", params.drive_topic1);
    node->get_parameter("state_topic1", params.state_topic1);
    node->get_parameter("scan_topic1", params.scan_topic1);
    node->get_parameter("mass1", params.p1.mass);
    node->get_parameter("l_r1", params.p1.l_r);
    node->get_parameter("l_f1", params.p1.l_f);
    node->get_parameter("I_z1", params.p1.I_z);
    node->get_parameter("h_cg1", params.p1.h_cg);
    node->get_parameter("cs_f1", params.p1.cs_f);
    node->get_parameter("cs_r1", params.p1.cs_r);
    node->get_parameter("B_f1", params.p1.Bf);
    node->get_parameter("C_f1", params.p1.Cf);
    node->get_parameter("D_f1", params.p1.Df);
    node->get_parameter("B_r1", params.p1.Br);
    node->get_parameter("C_r1", params.p1.Cr);
    node->get_parameter("D_r1", params.p1.Dr);
    node->get_parameter("steer_max1", params.p1.steer_max);
    node->get_parameter("steer_vel_max1", params.p1.steer_vel_max);
    node->get_parameter("speed_max1", params.p1.speed_max);
    node->get_parameter("accel_max1", params.p1.accel_max);
    node->get_parameter("decel_max1", params.p1.decel_max);
    node->get_parameter("jerk_max1", params.p1.jerk_max);


    return params;
    
};