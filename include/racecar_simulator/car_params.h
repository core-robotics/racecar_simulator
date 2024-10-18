#pragma once
struct CarParams {

    std::string drive_topic;
    std::string state_topic;
    std::string scan_topic;
    std::string collision_topic;
    int model_type;// 0 for single track model, 1 for pacejka tire model

    //for general car model
    double mass;
    double h_cg; // height of car's CG
    double l_f; // length from CG to front axle
    double l_r; // length from CG to rear axle
    double I_z; // moment of inertia about z axis from CG
    double friction_coeff;
    double steer_max; // max steering angle
    double steer_vel_max; // max steering velocity
    double speed_max; // max speed
    double accel_max; // max acceleration
    double decel_max; // max deceleration
    double jerk_max; // max jerk

    //for single track model
    double cs_f; // cornering stiffness coeff for front wheels
    double cs_r; // cornering stiffness coeff for rear wheels

    //for pacejka tire model
    double Bf; // Pacejka tire model parameter : stiffness factor
    double Cf; // Pacejka tire model parameter : shape factor
    double Df; // Pacejka tire model parameter : peak value of longitudinal force
    double Br;
    double Cr;
    double Dr;
};