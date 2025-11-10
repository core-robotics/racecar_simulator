#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <utility>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "control_msgs/msg/car_state.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "std_msgs/msg/bool.hpp"

#include "racecar_simulator/scan_simulator_2d.hpp"

using namespace racecar_simulator;

class RacecarSimulator : public rclcpp::Node {
private:
  // Timers
  rclcpp::TimerBase::SharedPtr simulator_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // Callback groups (separate compute vs publish)
  rclcpp::CallbackGroup::SharedPtr sim_group_;
  rclcpp::CallbackGroup::SharedPtr pub_group_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive0_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive1_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_path_sub_;

  // Publishers (BestEffort + KeepLast(1) to avoid blocking)
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan0_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan1_pub_;
  rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state0_pub_;
  rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state1_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision0_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision1_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom0_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom1_pub_;

  // State
  control_msgs::msg::CarState car_state0_, car_state1_;
  control_msgs::msg::CarState init_car_state0_;

  // Prebuilt messages for lightweight publish loop
  sensor_msgs::msg::LaserScan scan_msg_data0_, scan_msg_data1_;
  nav_msgs::msg::Odometry odom_msg0_, odom_msg1_;
  std_msgs::msg::Bool collision_msg0_, collision_msg1_;

  // Scan simulator
  ScanSimulator2D scan_simulator_;

  // Parameters
  double simulator_frequency_{1000.0};
  double pub_frequency_{40.0};
  int    scan_beams_{1080};
  double scan_fov_{2.0 * M_PI};
  double scan_std_dev_{0.01};
  double map_free_threshold_{0.2};

  bool detect_car_mode_{false};
  bool state_noise_mode_{false};
  bool scan_noise_mode_{false};

  // Car model parameters
  struct CarParams {
    double mass, l_r, l_f, I_z;
    double B_f, C_f, D_f, B_r, C_r, D_r;
    double steer_max, steer_vel_max;
    double speed_max, accel_max, decel_max, jerk_max;
  } car0_params_{}, car1_params_{};

  int vehicle_model0_{1}, vehicle_model1_{1};
  std::string drive_topic0_, state_topic0_, drive_topic1_, state_topic1_, scan_topic0_, scan_topic1_;

  // Desired inputs
  double desired_accel0_{0.0}, desired_steer_ang0_{0.0};
  double desired_accel1_{0.0}, desired_steer_ang1_{0.0};

  // Map buffers
  bool map_exists_{false};
  nav_msgs::msg::OccupancyGrid original_map_;
  nav_msgs::msg::OccupancyGrid current_map_;

  // Scan buffers (pre-allocated)
  std::vector<float> scan_data_float0_, scan_data_float1_;

  // Collision ROI in laser frame
  float x_min_ = -0.32f, x_max_ = 0.08f, y_min_ = -0.20f, y_max_ = 0.20f;

  // Path start pose handling
  bool receive_start_pose_{false};
  bool is_pose_init_{false};

public:
  RacecarSimulator()
  : Node("racecar_simulator"),
    scan_simulator_(scan_beams_, scan_fov_, scan_std_dev_) {
    // General parameters
    this->declare_parameter("simulator_frequency", 1000.0);
    this->declare_parameter("pub_frequency", 40.0);
    this->declare_parameter("scan_beams", 1080);
    this->declare_parameter("scan_field_of_view", 2.0 * M_PI);
    this->declare_parameter("scan_std_dev", 0.01);
    this->declare_parameter("map_free_threshold", 0.2);
    this->declare_parameter("detect_car_mode", false);
    this->declare_parameter("state_noise_mode", false);
    this->declare_parameter("scan_noise_mode", false);

    this->get_parameter("simulator_frequency", simulator_frequency_);
    this->get_parameter("pub_frequency", pub_frequency_);
    this->get_parameter("scan_beams", scan_beams_);
    this->get_parameter("scan_field_of_view", scan_fov_);
    this->get_parameter("scan_std_dev", scan_std_dev_);
    this->get_parameter("map_free_threshold", map_free_threshold_);
    this->get_parameter("detect_car_mode", detect_car_mode_);
    this->get_parameter("state_noise_mode", state_noise_mode_);
    this->get_parameter("scan_noise_mode", scan_noise_mode_);

    // Car0 parameters
    this->declare_parameter("vehicle_model0", 1);
    this->declare_parameter("drive_topic0", "ackermann_cmd0");
    this->declare_parameter("state_topic0", "state0");
    this->declare_parameter("scan_topic0", "scan0");
    this->declare_parameter("mass0", 3.5);
    this->declare_parameter("l_r0", 0.17145);
    this->declare_parameter("l_f0", 0.17145);
    this->declare_parameter("I_z0", 0.04712);
    this->declare_parameter("B_f0", 1.5);
    this->declare_parameter("C_f0", 1.5);
    this->declare_parameter("D_f0", 30.0);
    this->declare_parameter("B_r0", 1.5);
    this->declare_parameter("C_r0", 1.5);
    this->declare_parameter("D_r0", 30.0);
    this->declare_parameter("steer_max0", 4.0);
    this->declare_parameter("steer_vel_max0", 4.0);
    this->declare_parameter("speed_max0", 10.0);
    this->declare_parameter("accel_max0", 40.0);
    this->declare_parameter("decel_max0", 40.0);
    this->declare_parameter("jerk_max0", 100.0);

    this->get_parameter("vehicle_model0", vehicle_model0_);
    this->get_parameter("drive_topic0", drive_topic0_);
    this->get_parameter("state_topic0", state_topic0_);
    this->get_parameter("scan_topic0", scan_topic0_);
    this->get_parameter("mass0", car0_params_.mass);
    this->get_parameter("l_r0", car0_params_.l_r);
    this->get_parameter("l_f0", car0_params_.l_f);
    this->get_parameter("I_z0", car0_params_.I_z);
    this->get_parameter("B_f0", car0_params_.B_f);
    this->get_parameter("C_f0", car0_params_.C_f);
    this->get_parameter("D_f0", car0_params_.D_f);
    this->get_parameter("B_r0", car0_params_.B_r);
    this->get_parameter("C_r0", car0_params_.C_r);
    this->get_parameter("D_r0", car0_params_.D_r);
    this->get_parameter("steer_max0", car0_params_.steer_max);
    this->get_parameter("steer_vel_max0", car0_params_.steer_vel_max);
    this->get_parameter("speed_max0", car0_params_.speed_max);
    this->get_parameter("accel_max0", car0_params_.accel_max);
    this->get_parameter("decel_max0", car0_params_.decel_max);
    this->get_parameter("jerk_max0", car0_params_.jerk_max);

    // Car1 parameters
    this->declare_parameter("vehicle_model1", 1);
    this->declare_parameter("drive_topic1", "ackermann_cmd1");
    this->declare_parameter("state_topic1", "state1");
    this->declare_parameter("scan_topic1", "scan1");
    this->declare_parameter("mass1", 3.5);
    this->declare_parameter("l_r1", 0.17145);
    this->declare_parameter("l_f1", 0.17145);
    this->declare_parameter("I_z1", 0.04712);
    this->declare_parameter("B_f1", 1.5);
    this->declare_parameter("C_f1", 1.5);
    this->declare_parameter("D_f1", 30.0);
    this->declare_parameter("B_r1", 1.5);
    this->declare_parameter("C_r1", 1.5);
    this->declare_parameter("D_r1", 30.0);
    this->declare_parameter("steer_max1", 0.4);
    this->declare_parameter("steer_vel_max1", 0.041);
    this->declare_parameter("speed_max1", 10.0);
    this->declare_parameter("accel_max1", 4.0);
    this->declare_parameter("decel_max1", 4.0);
    this->declare_parameter("jerk_max1", 1.0);

    this->get_parameter("vehicle_model1", vehicle_model1_);
    this->get_parameter("drive_topic1", drive_topic1_);
    this->get_parameter("state_topic1", state_topic1_);
    this->get_parameter("scan_topic1", scan_topic1_);
    this->get_parameter("mass1", car1_params_.mass);
    this->get_parameter("l_r1", car1_params_.l_r);
    this->get_parameter("l_f1", car1_params_.l_f);
    this->get_parameter("I_z1", car1_params_.I_z);
    this->get_parameter("B_f1", car1_params_.B_f);
    this->get_parameter("C_f1", car1_params_.C_f);
    this->get_parameter("D_f1", car1_params_.D_f);
    this->get_parameter("B_r1", car1_params_.B_r);
    this->get_parameter("C_r1", car1_params_.C_r);
    this->get_parameter("D_r1", car1_params_.D_r);
    this->get_parameter("steer_max1", car1_params_.steer_max);
    this->get_parameter("steer_vel_max1", car1_params_.steer_vel_max);
    this->get_parameter("speed_max1", car1_params_.speed_max);
    this->get_parameter("accel_max1", car1_params_.accel_max);
    this->get_parameter("decel_max1", car1_params_.decel_max);
    this->get_parameter("jerk_max1", car1_params_.jerk_max);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Callback groups
    sim_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Timers with nanosecond precision
    auto simulator_period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / simulator_frequency_));
    auto pub_period_ns       = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / pub_frequency_));

    simulator_timer_ = this->create_wall_timer(
      simulator_period_ns, std::bind(&RacecarSimulator::simulatorLoop, this), sim_group_);

    pub_timer_ = this->create_wall_timer(
      pub_period_ns, std::bind(&RacecarSimulator::pubLoop, this), pub_group_);

	auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
	auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto sub_qos   = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();


    // Subscriptions (reliable control/map/path)
    init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", map_qos,
      std::bind(&RacecarSimulator::car0RvizCallback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", map_qos,
      std::bind(&RacecarSimulator::car1RvizCallback, this, std::placeholders::_1));

    drive0_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic0_, sub_qos,
      std::bind(&RacecarSimulator::drive0Callback, this, std::placeholders::_1));

    drive1_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic1_, sub_qos,
      std::bind(&RacecarSimulator::drive1Callback, this, std::placeholders::_1));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", map_qos,
      std::bind(&RacecarSimulator::mapCallback, this, std::placeholders::_1));

    center_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "center_path", map_qos,
      std::bind(&RacecarSimulator::centerPathCallback, this, std::placeholders::_1));



    scan0_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic0_, pub_qos);
    scan1_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic1_, pub_qos);
    state0_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic0_, pub_qos);
    state1_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic1_, pub_qos);
    collision0_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision0", pub_qos);
    collision1_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision1", pub_qos);
    odom0_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom0", pub_qos);
    odom1_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom1", pub_qos);
    map_pub_   = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_out", pub_qos);

    // Pre-allocate scan buffers
    scan_data_float0_.assign(static_cast<size_t>(scan_beams_), 0.0f);
    scan_data_float1_.assign(static_cast<size_t>(scan_beams_), 0.0f);

    // Init collision flags
    collision_msg0_.data = false;
    collision_msg1_.data = false;

    RCLCPP_INFO(this->get_logger(), "Racecar simulator initialized (stable 100Hz mode).");
  }

  // Simulation loop: compute-heavy only
  void simulatorLoop() {
    setInput(car_state0_, desired_accel0_, desired_steer_ang0_, car0_params_);
    setInput(car_state1_, desired_accel1_, desired_steer_ang1_, car1_params_);

    updateState();

    if (receive_start_pose_ && !is_pose_init_) {
      car_state0_.px = init_car_state0_.px;
      car_state0_.py = init_car_state0_.py;
      car_state0_.yaw = init_car_state0_.yaw;
      car_state1_.px = 100.0;
      car_state1_.py = 100.0;
      car_state1_.yaw = 0.0;
      is_pose_init_ = true;
    }

    setTF();

    if (map_exists_) {
      current_map_ = original_map_;
      prepareScan(car_state0_, "laser_model0", scan_data_float0_, scan_msg_data0_);
      prepareScan(car_state1_, "laser_model1", scan_data_float1_, scan_msg_data1_);
      collision_msg0_.data = checkCollision(scan_msg_data0_);
      collision_msg1_.data = checkCollision(scan_msg_data1_);
    }

    prepareOdom(car_state0_, "base_link0", "odom0", odom_msg0_);
    prepareOdom(car_state1_, "base_link1", "odom1", odom_msg1_);
  }

  // Publish loop: lightweight publish only
  void pubLoop() {
    state0_pub_->publish(car_state0_);
    state1_pub_->publish(car_state1_);
    odom0_pub_->publish(odom_msg0_);
    odom1_pub_->publish(odom_msg1_);
    if (map_exists_) {
      scan0_pub_->publish(scan_msg_data0_);
      scan1_pub_->publish(scan_msg_data1_);
      collision0_pub_->publish(collision_msg0_);
      collision1_pub_->publish(collision_msg1_);
    }
  }

  // TF helpers
  void publishTransform(const std::string &frame_id, const std::string &child_frame_id,
                        double x, double y, double yaw) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = frame_id;
    t.child_frame_id = child_frame_id;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    t.transform.rotation = tf2::toMsg(q);

    if (std::isnan(t.transform.translation.x) || std::isnan(t.transform.translation.y) ||
        std::isnan(t.transform.rotation.x) || std::isnan(t.transform.rotation.y) ||
        std::isnan(t.transform.rotation.z) || std::isnan(t.transform.rotation.w)) {
      return;
    }
    tf_broadcaster_->sendTransform(t);
  }

  void setTF() {
    publishTransform("map", "base_link0", car_state0_.px, car_state0_.py, car_state0_.yaw);
    publishTransform("front_left_hinge0", "front_left_wheel0", 0.0, 0.0, car_state0_.steer);
    publishTransform("front_right_hinge0", "front_right_wheel0", 0.0, 0.0, car_state0_.steer);
	
    publishTransform("map", "base_link1", car_state1_.px, car_state1_.py, car_state1_.yaw);
    publishTransform("front_left_hinge1", "front_left_wheel1", 0.0, 0.0, car_state1_.steer);
    publishTransform("front_right_hinge1", "front_right_wheel1", 0.0, 0.0, car_state1_.steer);
  }

  // Vehicle input and models
  void setInput(control_msgs::msg::CarState &state, double desired_accel, double desired_steer_ang, const CarParams &p) {
    double dt = 1.0 / simulator_frequency_;

    double steer_diff = desired_steer_ang - state.steer;
    double steer_change_max = p.steer_vel_max * dt;
    if (std::abs(steer_diff) > steer_change_max) {
      state.steer += steer_change_max * (steer_diff / std::abs(steer_diff));
    } else {
      state.steer += steer_diff;
    }
    state.steer = std::clamp(state.steer, -p.steer_max, p.steer_max);

    double accel_diff = desired_accel - state.accel;
    double accel_change_max = p.jerk_max * dt;
    if (std::abs(accel_diff) > accel_change_max) {
      state.accel += accel_change_max * (accel_diff / std::abs(accel_diff));
    } else {
      state.accel += accel_diff;
    }
    state.accel = std::clamp(state.accel, -p.decel_max, p.accel_max);

    if (std::isnan(state.steer) || std::isnan(state.accel)) {
      state.steer = 0.0;
      state.accel = 0.0;
    }
  }

  control_msgs::msg::CarState update_k(const control_msgs::msg::CarState start, double accel, double steer_vel, const CarParams &p, double dt) {
    control_msgs::msg::CarState end;
    double x_dot = start.v * std::cos(start.yaw);
    double y_dot = start.v * std::sin(start.yaw);
    double v_dot = accel;
    double steer_angle_dot = steer_vel;
    double theta_dot = start.v / (p.l_f + p.l_r) * std::tan(start.steer);

    end.px = start.px + x_dot * dt;
    end.py = start.py + y_dot * dt;
    end.yaw = start.yaw + theta_dot * dt;
    end.v = start.v + v_dot * dt;
    end.steer = start.steer + steer_angle_dot * dt;
    end.omega = 0.0;
    end.slip_angle = 0.0;

    if (end.yaw > M_PI) end.yaw -= 2 * M_PI;
    else if (end.yaw < -M_PI) end.yaw += 2 * M_PI;

    return end;
  }

  control_msgs::msg::CarState updateStateSingleTrack(control_msgs::msg::CarState &start, const CarParams &p) {
    if (std::abs(start.v) < 0.1) {
      return update_k(start, start.accel, start.steer_vel, p, 1.0 / simulator_frequency_);
    }
    double g = 9.81;
    double h_cg = 0.074;
    double friction_coeff = 0.8;
    double cs_f = 4.718;
    double cs_r = 5.74562;
    double dt = 1.0 / simulator_frequency_;

    double x_dot = start.v * std::cos(start.yaw + start.slip_angle);
    double y_dot = start.v * std::sin(start.yaw + start.slip_angle);
    double v_dot = start.accel;

    double rear_val = g * p.l_r - start.accel * h_cg;
    double front_val = g * p.l_f + start.accel * h_cg;

    double omega_dot = (friction_coeff * p.mass / (p.I_z * (p.l_f + p.l_r))) *
                       (p.l_f * cs_f * start.steer * (rear_val) +
                        start.slip_angle * (p.l_r * cs_r * (front_val) - p.l_f * cs_f * (rear_val)) -
                        (start.omega / start.v) * (std::pow(p.l_f, 2) * cs_f * (rear_val) + std::pow(p.l_r, 2) * cs_r * (front_val)));

    double slip_angle_dot = (friction_coeff / (start.v * (p.l_r + p.l_f))) *
                              (cs_f * start.steer * rear_val -
                               start.slip_angle * (cs_r * front_val + cs_f * rear_val) +
                               (start.omega / start.v) * (cs_r * p.l_r * front_val - cs_f * p.l_f * rear_val)) -
                            start.omega;

    control_msgs::msg::CarState end;
    end.px = start.px + x_dot * dt;
    end.py = start.py + y_dot * dt;
    end.yaw = start.yaw + start.omega * dt;
    end.slip_angle = start.slip_angle + slip_angle_dot * dt;

    end.v = start.v + v_dot * dt;
    end.vx = start.v * std::cos(start.slip_angle);
    end.vy = start.v * std::sin(start.slip_angle);
    end.omega = start.omega + omega_dot * dt;

    end.a = start.accel;
    end.ax = start.a * std::cos(start.slip_angle) - start.v * start.omega * std::sin(start.slip_angle);
    end.ay = start.a * std::sin(start.slip_angle) + start.v * start.omega * std::cos(start.slip_angle);

    end.accel = start.accel;
    end.steer = start.steer;

    end.v = std::clamp(end.v, -p.speed_max, p.speed_max);

    if (end.yaw > M_PI) end.yaw -= 2 * M_PI;
    else if (end.yaw < -M_PI) end.yaw += 2 * M_PI;

    while (end.slip_angle > M_PI) end.slip_angle -= 2 * M_PI;
    while (end.slip_angle < -M_PI) end.slip_angle += 2 * M_PI;

    return end;
  }

  control_msgs::msg::CarState updateStatePacejka(control_msgs::msg::CarState &start, const CarParams &p) {
    if (std::abs(start.v) < 1.0e-8) {
      return update_k(start, start.accel, start.steer_vel, p, 1.0 / simulator_frequency_);
    }
    control_msgs::msg::CarState end;
    double dt = 1.0 / simulator_frequency_;
    double a_f = -std::atan2(start.vy + p.l_f * start.omega, start.vx) + start.steer;
    double F_fy = p.D_f * std::sin(p.C_f * std::atan(p.B_f * a_f));
    double a_r = -std::atan2(start.vy - p.l_r * start.omega, start.vx);
    double F_ry = p.D_r * std::sin(p.C_r * std::atan(p.B_r * a_r));

    double x_dot = start.v * std::cos(start.yaw + start.slip_angle);
    double y_dot = start.v * std::sin(start.yaw + start.slip_angle);
    double yaw_dot = start.omega;
    double slip_angle_dot = ((F_fy + F_ry) / (p.mass * start.v)) - start.omega;
    double v_dot = start.a;
    double omega_dot = (p.l_f * F_fy * std::cos(start.steer) - p.l_r * F_ry) / p.I_z;

    end.px = start.px + x_dot * dt;
    end.py = start.py + y_dot * dt;
    end.yaw = start.yaw + yaw_dot * dt;
    end.slip_angle = start.slip_angle + slip_angle_dot * dt;

    end.v = start.v + v_dot * dt;
    end.vx = start.v * std::cos(start.slip_angle);
    end.vy = start.v * std::sin(start.slip_angle);
    end.omega = start.omega + omega_dot * dt;

    end.a = start.accel;
    end.ax = start.a * std::cos(start.slip_angle) - start.v * start.omega * std::sin(start.slip_angle);
    end.ay = start.a * std::sin(start.slip_angle) + start.v * start.omega * std::cos(start.slip_angle);

    end.accel = start.accel;
    end.steer = start.steer;

    end.v = std::clamp(end.v, -p.speed_max, p.speed_max);

    if (end.yaw > M_PI) end.yaw -= 2 * M_PI;
    else if (end.yaw < -M_PI) end.yaw += 2 * M_PI;

    while (end.slip_angle > M_PI) end.slip_angle -= 2 * M_PI;
    while (end.slip_angle < -M_PI) end.slip_angle += 2 * M_PI;

    if (state_noise_mode_) {
      auto n = [this](double s){ return gen_noise(s); };
      end.px += n(0.0001); end.py += n(0.0001); end.yaw += n(0.0001);
      end.v  += n(0.0001); end.vx += n(0.0001); end.vy += n(0.0001);
      end.omega += n(0.0001); end.a += n(0.0001); end.ax += n(0.0001); end.ay += n(0.0001);
      end.accel += n(0.0001); end.steer += n(0.0001); end.slip_angle += n(0.01);
    }

    return end;
  }

  void updateState() {
    if (vehicle_model0_ == 0) car_state0_ = updateStateSingleTrack(car_state0_, car0_params_);
    else if (vehicle_model0_ == 1) car_state0_ = updateStatePacejka(car_state0_, car0_params_);

    if (vehicle_model1_ == 0) car_state1_ = updateStateSingleTrack(car_state1_, car1_params_);
    else if (vehicle_model1_ == 1) car_state1_ = updateStatePacejka(car_state1_, car1_params_);
  }

  // Callbacks
  void car0RvizCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);

    car_state0_.px = msg->pose.pose.position.x;
    car_state0_.py = msg->pose.pose.position.y;
    car_state0_.yaw = yaw;
    car_state0_.v = 0.0;
    car_state0_.a = 0.0;
    car_state0_.accel = 0.0;
    desired_accel0_ = 0.0;
    car_state0_.steer = 0.0;

    publishTransform("map", "base_link0", car_state0_.px, car_state0_.py, car_state0_.yaw);
  }

  void car1RvizCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.orientation.x,
                      msg->pose.orientation.y,
                      msg->pose.orientation.z,
                      msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);

    car_state1_.px = msg->pose.position.x;
    car_state1_.py = msg->pose.position.y;
    car_state1_.yaw = yaw;
    car_state1_.v = 0.0;
    car_state1_.a = 0.0;
    car_state1_.accel = 0.0;
    desired_accel1_ = 0.0;
    car_state1_.steer = 0.0;

    publishTransform("map", "base_link1", car_state1_.px, car_state1_.py, car_state1_.yaw);
  }

  void drive0Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    desired_accel0_ = msg->drive.acceleration;
    desired_steer_ang0_ = msg->drive.steering_angle;
  }

  void drive1Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    desired_accel1_ = msg->drive.acceleration;
    desired_steer_ang1_ = msg->drive.steering_angle;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    size_t height = msg->info.height;
    size_t width = msg->info.width;
    double resolution = msg->info.resolution;

    Pose2D origin;
    origin.x = msg->info.origin.position.x;
    origin.y = msg->info.origin.position.y;

    tf2::Quaternion quat(msg->info.origin.orientation.x,
                         msg->info.origin.orientation.y,
                         msg->info.origin.orientation.z,
                         msg->info.origin.orientation.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw; mat.getRPY(roll, pitch, yaw);
    origin.theta = yaw;

    if (msg->data.size() != height * width) {
      RCLCPP_ERROR(this->get_logger(), "Map data size mismatch.");
      return;
    }

    std::vector<double> map(msg->data.size(), 0.5);
    for (size_t i = 0; i < msg->data.size(); i++) {
      if (msg->data[i] > 100 || msg->data[i] < 0) {
        map[i] = 0.5;
      } else {
        map[i] = msg->data[i] / 100.0;
      }
    }

    scan_simulator_.set_map(map, height, width, resolution, origin, map_free_threshold_);
    original_map_ = *msg;
    map_exists_ = true;
  }

  void centerPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.size() < 2) return;

    tf2::Quaternion q(msg->poses[1].pose.orientation.x,
                      msg->poses[1].pose.orientation.y,
                      msg->poses[1].pose.orientation.z,
                      msg->poses[1].pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);

    init_car_state0_.px = msg->poses[0].pose.position.x;
    init_car_state0_.py = msg->poses[0].pose.position.y;
    init_car_state0_.yaw = yaw;

    receive_start_pose_ = true;
  }

  // Scan / collision / odom preparation
  void prepareScan(const control_msgs::msg::CarState &state,
                   const std::string &scan_frame,
                   std::vector<float> &scan_data_float,
                   sensor_msgs::msg::LaserScan &scan_msg_out
				) {
    if (!map_exists_) return;

    Pose2D scan_pose;
    const double scan_offset = 0.12;
    if (scan_noise_mode_) {
      scan_pose.x = state.px + scan_offset * std::cos(state.yaw) + gen_noise(0.001);
      scan_pose.y = state.py + scan_offset * std::sin(state.yaw) + gen_noise(0.001);
      scan_pose.theta = state.yaw + gen_noise(0.01);
    } else {
      scan_pose.x = state.px + scan_offset * std::cos(state.yaw);
      scan_pose.y = state.py + scan_offset * std::sin(state.yaw);
      scan_pose.theta = state.yaw;
    }

    std::vector<double> scan_data = scan_simulator_.scan(scan_pose);

	scan_data_float.resize(scan_data.size());

	for (size_t i = 0; i < scan_data.size(); i++)
	{
		scan_data_float[i] = scan_data[i];
	}

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = this->get_clock()->now();
    scan_msg.header.frame_id = scan_frame;
    scan_msg.angle_min = -scan_simulator_.get_field_of_view() / 2.0;
    scan_msg.angle_max =  scan_simulator_.get_field_of_view() / 2.0;
    scan_msg.angle_increment = scan_simulator_.get_angle_increment();
    scan_msg.range_max = 10.0;
    scan_msg.range_min = 0.1;
    scan_msg.ranges = scan_data_float;
    // scan_msg.intensities = std::vector<float>(N, 0.0f);
	scan_msg.intensities = std::vector<float>(scan_data.size(), 0.0);
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 1.0 / pub_frequency_;

    scan_msg_out = scan_msg;
  }

  bool checkCollision(const sensor_msgs::msg::LaserScan &scan) {
    if (scan.ranges.empty()) return false;
    for (size_t i = 0; i < scan.ranges.size(); i++) {
      float angle = static_cast<float>(scan.angle_min + i * scan.angle_increment);
      float x = scan.ranges[i] * std::cos(angle);
      float y = scan.ranges[i] * std::sin(angle);
      if (x > x_min_ && x < x_max_ && y > y_min_ && y < y_max_) {
        return true;
      }
    }
    return false;
  }

  void prepareOdom(const control_msgs::msg::CarState &state,
                   const std::string &frame_id,
                   const std::string &child_frame_id,
                   nav_msgs::msg::Odometry &odom_out) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = state.px;
    odom.pose.pose.position.y = state.py;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state.yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = state.vx;
    odom.twist.twist.linear.y = state.vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = state.omega;

    odom_out = std::move(odom);
  }

  // Utilities
  double gen_noise(double std_dev) {
    static thread_local std::mt19937 gen(std::random_device{}());
    std::normal_distribution<double> dist(0.0, std_dev);
    return dist(gen);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RacecarSimulator>();

  // Default-constructed MultiThreadedExecutor is portable across distros
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
