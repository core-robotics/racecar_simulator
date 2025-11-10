#include <chrono>
#include <functional>
#include <memory>
// #include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>   // NEW: for std::transform
#include <random>      // NEW: for noise generation
#include <cmath>       // NEW: for trigonometric functions

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
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"

using namespace std::chrono_literals; // Use chrono literals for timing
using namespace racecar_simulator;

class RacecarSimulator : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr simulator_timer_;
	rclcpp::TimerBase::SharedPtr pub_timer_;
	rclcpp::TimerBase::SharedPtr scan_timer_;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive0_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive1_sub_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_path_sub_;

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan0_pub_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan1_pub_;
	rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state0_pub_;
	rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state1_pub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision0_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision1_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom0_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom1_pub_;
	// rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu0_pub_;
	// rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub_;

	control_msgs::msg::CarState car_state0_, car_state1_;

	ScanSimulator2D scan_simulator_;
	double map_free_threshold;

	struct CarParams
	{
		double mass, l_r, l_f, I_z;
		double B_f, C_f, D_f, B_r, C_r, D_r;
		double steer_max, steer_vel_max;
		double speed_max, accel_max, decel_max, jerk_max;
	};
	CarParams car0_params_, car1_params_;

	int vehicle_model0_, vehicle_model1_;
	std::string drive_topic0_, state_topic0_, drive_topic1_, state_topic1_, scan_topic0_, scan_topic1_;
	std::string pgm_file_path_, yaml_file_path_;
	double simulator_frequency_, pub_frequency_, scan_frequency_;
	bool detect_car_mode_ = false;
	bool state_noise_mode_ = false;
	bool scan_noise_mode_ = false;

	bool use_car1_ = true;

	double desired_speed0_, desired_accel0_, desired_steer_ang0_;
	double desired_speed1_, desired_accel1_, desired_steer_ang1_;
	double scan_fov_, scan_std_dev_;
	int scan_beams_;
	double map_free_threshold_;
	std::vector<float> scan_data_float0_, scan_data_float1_;
	sensor_msgs::msg::LaserScan scan_msg_data0_, scan_msg_data1_;

	bool map_exists_ = false;
	nav_msgs::msg::OccupancyGrid original_map_;
	nav_msgs::msg::OccupancyGrid current_map_;

	bool car0_collision_ = false;
	bool car1_collision_ = false;

	std::vector<std::pair<float, float>> scan_coordinates;
	float x_min = -0.32;
	float x_max = 0.08;
	float y_min = -0.2;
	float y_max = 0.2;

	bool receive_start_pose_ = false;
	bool is_pose_init_ = false;
	control_msgs::msg::CarState init_car_state0_;
	std::mt19937 rng_{std::random_device{}()};
	std::normal_distribution<double> n01_{0.0, 1.0};

public:
	RacecarSimulator()
		: Node("racecar_simulator")
	{
		// Load parameters
		// Params params = load_parameters(this);
		// General parameters
		this->declare_parameter("simulator_frequency", 200.0);
		this->declare_parameter("pub_frequency", 50.0);
		this->declare_parameter("scan_frequency", 40.0);
		this->declare_parameter("scan_beams", 1080);
		this->declare_parameter("scan_field_of_view", 2.0 * M_PI);
		this->declare_parameter("scan_std_dev", 0.01);
		this->declare_parameter("map_free_threshold", 0.2);
		this->declare_parameter("detect_car_mode", false);
		this->declare_parameter("state_noise_mode", false);
		this->declare_parameter("scan_noise_mode", false);
		// NEW: parameter to control whether car1 is used
		this->declare_parameter("use_car1", true);

		this->get_parameter("simulator_frequency", simulator_frequency_);
		this->get_parameter("pub_frequency", pub_frequency_);
		this->get_parameter("scan_frequency", scan_frequency_);
		this->get_parameter("scan_beams", scan_beams_);
		this->get_parameter("scan_field_of_view", scan_fov_);
		this->get_parameter("scan_std_dev", scan_std_dev_);
		this->get_parameter("map_free_threshold", map_free_threshold_);
		this->get_parameter("detect_car_mode", detect_car_mode_);
		this->get_parameter("state_noise_mode", state_noise_mode_);
		this->get_parameter("scan_noise_mode", scan_noise_mode_);
		// NEW: load use_car1 (default true to preserve behavior)
		this->get_parameter("use_car1", use_car1_);

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

		// Convert frequencies to durations
		auto simulator_period = std::chrono::duration<double>(1.0 / simulator_frequency_);
		// auto pub_period = std::chrono::duration<double>(1.0 / pub_frequency_);
		auto scan_period = std::chrono::duration<double>(1.0 / scan_frequency_);

		// Create publishers and subscribers
		
		simulator_timer_ = this->create_wall_timer(
			simulator_period,
			std::bind(&RacecarSimulator::simulatorLoop, this));
			
		// pub_timer_ = this->create_wall_timer(
		// 	pub_period,
		// 	std::bind(&RacecarSimulator::pubLoop, this));
				
		scan_timer_ = this->create_wall_timer(
			scan_period,
			std::bind(&RacecarSimulator::scanLoop, this));

					
		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		auto r_t_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
		auto r_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
		auto b_qos   = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
		// auto s_qos  = rclcpp::SensorDataQoS().best_effort().keep_last(1);

		init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", r_qos, std::bind(&RacecarSimulator::car0RvizCallback, this, std::placeholders::_1));

		// CHANGED: Create car1 endpoints only if use_car1_ is true
		if (use_car1_) { // NEW
			goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				"goal_pose", r_qos, std::bind(&RacecarSimulator::car1RvizCallback, this, std::placeholders::_1));
		}

		drive0_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic0_, b_qos, std::bind(&RacecarSimulator::drive0Callback, this, std::placeholders::_1));

		// CHANGED: Create car1 drive sub only if enabled
		if (use_car1_) { // NEW
			drive1_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
				drive_topic1_, b_qos, std::bind(&RacecarSimulator::drive1Callback, this, std::placeholders::_1));
		}

		map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
			"map", r_t_qos, std::bind(&RacecarSimulator::mapCallback, this, std::placeholders::_1));

		center_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
			"center_path", r_t_qos, std::bind(&RacecarSimulator::centerPathCallback, this, std::placeholders::_1));

		scan0_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic0_, r_qos);
		state0_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic0_, r_qos);
		collision0_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision0", r_qos);
		odom0_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom0", r_qos);

		// CHANGED: Create car1 pubs only if enabled
		if (use_car1_) { // NEW
			scan1_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic1_, r_qos);
			state1_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic1_, r_qos);
			collision1_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision1", r_qos);
			odom1_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom1", r_qos);
		}

		scan_simulator_ = ScanSimulator2D(scan_beams_, scan_fov_, scan_std_dev_);

		// Initialize simulator
		RCLCPP_INFO(this->get_logger(), "\nRacecar simulator initialized");
		RCLCPP_INFO(this->get_logger(), "\nSimulator frequency: %f Hz", simulator_frequency_);
		RCLCPP_INFO(this->get_logger(), "\nScan frequency: %f Hz", scan_frequency_);
		// RCLCPP_INFO(this->get_logger(), "\nPublish frequency: %f Hz", pub_frequency_);
		RCLCPP_INFO(this->get_logger(), "\nuse_car1: %s", use_car1_ ? "true" : "false"); 
		// RCLCPP_INFO(this->get_logger(), "\nvehicle_model0: %d", vehicle_model0_);
		// RCLCPP_INFO(this->get_logger(), "\nvehicle_model1: %d", vehicle_model1_);

		// //levinelobby
		// car_state0_.px = 0.688;
		// car_state0_.px = 0.688;
		// car_state0_.py = -0.906;
		// car_state0_.yaw = -70 * M_PI / 180;
	}

	// Simulator loop for updating car states
	void simulatorLoop()
	{
		if (receive_start_pose_==true&&is_pose_init_ == false)
		{
			car_state0_.px = init_car_state0_.px;
			car_state0_.py = init_car_state0_.py;
			car_state0_.yaw = init_car_state0_.yaw;

			// NEW: Only initialize car1 pose when enabled
			if (use_car1_) {
				car_state1_.px = 100;
				car_state1_.py = 100;
				car_state1_.yaw = 0;
			}
			is_pose_init_ = true;
		}

		// current_map_ = original_map_;
		// pub_scan(car_state0_, "laser_model0", scan0_pub_, scan_msg_data0_);
		state0Publisher();
		pub_colision(scan_msg_data0_, collision0_pub_);
		pub_odom(car_state0_, "base_link0", "odom0", odom0_pub_);

		// NEW: Entirely skip car1 publish path when disabled
		if (use_car1_) {
			// pub_scan(car_state1_, "laser_model1", scan1_pub_, scan_msg_data1_);
			state1Publisher();
			pub_colision(scan_msg_data1_, collision1_pub_);
			pub_odom(car_state1_, "base_link1", "odom1", odom1_pub_);
		}


		setInput(car_state0_, desired_accel0_, desired_steer_ang0_, car0_params_);
		// NEW: Skip car1 input update entirely when disabled
		if (use_car1_) {
			setInput(car_state1_, desired_accel1_, desired_steer_ang1_, car1_params_);
		}

		updateState();

		setTF();

	}

	// Publisher loop for broadcasting car states
	void pubLoop()
	{

		// if (receive_start_pose_==true&&is_pose_init_ == false)
		// {
		// 	car_state0_.px = init_car_state0_.px;
		// 	car_state0_.py = init_car_state0_.py;
		// 	car_state0_.yaw = init_car_state0_.yaw;

		// 	// NEW: Only initialize car1 pose when enabled
		// 	if (use_car1_) {
		// 		car_state1_.px = 100;
		// 		car_state1_.py = 100;
		// 		car_state1_.yaw = 0;
		// 	}
		// 	is_pose_init_ = true;
		// }

		// // current_map_ = original_map_;
		// // pub_scan(car_state0_, "laser_model0", scan0_pub_, scan_msg_data0_);
		// state0Publisher();
		// pub_colision(scan_msg_data0_, collision0_pub_);
		// pub_odom(car_state0_, "base_link0", "odom0", odom0_pub_);

		// // NEW: Entirely skip car1 publish path when disabled
		// if (use_car1_) {
		// 	// pub_scan(car_state1_, "laser_model1", scan1_pub_, scan_msg_data1_);
		// 	state1Publisher();
		// 	pub_colision(scan_msg_data1_, collision1_pub_);
		// 	pub_odom(car_state1_, "base_link1", "odom1", odom1_pub_);
		// }
		return;
	}

	// Timer callback for scan publishing
	void scanLoop()
	{
		pub_scan(car_state0_, "laser_model0", scan0_pub_, scan_msg_data0_);

		// NEW: Skip car1 scan publish when disabled
		if (use_car1_) {
			pub_scan(car_state1_, "laser_model1", scan1_pub_, scan_msg_data1_);
		}
	}

	// Publish transform between frames
	void publishTransform(const std::string &frame_id, const std::string &child_frame_id,
						  double x, double y, double yaw)
	{
		geometry_msgs::msg::TransformStamped t;

		t.header.stamp = this->get_clock()->now();
		t.header.frame_id = frame_id;
		t.child_frame_id = child_frame_id;

		t.transform.translation.x = x;
		t.transform.translation.y = y;
		t.transform.translation.z = 0.0;

		tf2::Quaternion q;
		q.normalize();
		q.setRPY(0, 0, yaw);
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();
		if (std::isnan(t.transform.translation.x) || std::isnan(t.transform.translation.y) ||
			std::isnan(t.transform.translation.z) || std::isnan(t.transform.rotation.x) ||
			std::isnan(t.transform.rotation.y) || std::isnan(t.transform.rotation.z) ||
			std::isnan(t.transform.rotation.w))
		{
			RCLCPP_WARN(this->get_logger(), "Transformation contains NaN values and will be ignored.");
			return;
		}

		// Send the transformation
		tf_broadcaster_->sendTransform(t);
	}

	void setTF()
	{

		publishTransform("map", "base_link0", car_state0_.px, car_state0_.py, car_state0_.yaw);
		publishTransform("front_left_hinge0", "front_left_wheel0", 0.0, 0.0, car_state0_.steer);
		publishTransform("front_right_hinge0", "front_right_wheel0", 0.0, 0.0, car_state0_.steer);

		// NEW: car1 TF only when enabled
		if (use_car1_) {
			publishTransform("map", "base_link1", car_state1_.px, car_state1_.py, car_state1_.yaw);
			publishTransform("front_left_hinge1", "front_left_wheel1", 0.0, 0.0, car_state1_.steer);
			publishTransform("front_right_hinge1", "front_right_wheel1", 0.0, 0.0, car_state1_.steer);
		}
	}

	void updateState()
	{
		if (vehicle_model0_ == 0)
		{
			car_state0_ = updateStateSingleTrack(car_state0_, car0_params_);
		}
		else if (vehicle_model0_ == 1)
		{
			car_state0_ = updateStatePacejka(car_state0_, car0_params_);
		}
		else
		{
			std::cout << "Invalid vehicle model for car0" << std::endl;
		}

		// NEW: car1 update only when enabled; otherwise no computation is performed
		if (use_car1_) {
			if (vehicle_model1_ == 0)
			{
				car_state1_ = updateStateSingleTrack(car_state1_, car1_params_);
			}
			else if (vehicle_model1_ == 1)
			{
				car_state1_ = updateStatePacejka(car_state1_, car1_params_);
			}
			else
			{
				std::cout << "Invalid vehicle model for car1" << std::endl;
			}
		}
	}

	// Callback for initial pose of car0
	void car0RvizCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
		// Convert quaternion to Euler angles to extract yaw
		tf2::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);

		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		car_state0_.px = msg->pose.pose.position.x;
		car_state0_.py = msg->pose.pose.position.y;
		car_state0_.yaw = yaw;
		car_state0_.v = 0.0;
		car_state0_.a = 0.0;
		car_state0_.accel = 0.0;
		desired_accel0_ = 0.0;
		car_state0_.steer = 0.0;

		publishTransform("map", "base_link0", car_state0_.px, car_state0_.py, car_state0_.yaw);

		RCLCPP_INFO(this->get_logger(), "\nCar0 x: %f, y: %f, yaw: %f", car_state0_.px, car_state0_.py, car_state0_.yaw);
	}

	// Callback for initial pose of car1
	void car1RvizCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		// Optional guard: if no car1, ignore. This does not change behavior when subscriptions are not created.
		if (!use_car1_) return; // NEW (defensive, callback won't be connected when disabled)

		// Convert quaternion to Euler angles to extract yaw
		tf2::Quaternion q(
			msg->pose.orientation.x,
			msg->pose.orientation.y,
			msg->pose.orientation.z,
			msg->pose.orientation.w);

		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		car_state1_.px = msg->pose.position.x;
		car_state1_.py = msg->pose.position.y;
		car_state1_.yaw = yaw;
		car_state1_.v = 0.0;
		car_state1_.a = 0.0;
		car_state1_.accel = 0.0;
		desired_accel1_ = 0.0;
		car_state1_.steer = 0.0;

		publishTransform("map", "base_link1", car_state1_.px, car_state1_.py, car_state1_.yaw);
		RCLCPP_INFO(this->get_logger(), "\nCar1 x: %f, y: %f, yaw: %f", car_state1_.px, car_state1_.py, car_state1_.yaw);
	}

	// Callback for drive command of car0
	void drive0Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
	{
		desired_accel0_ = msg->drive.acceleration;
		desired_steer_ang0_ = msg->drive.steering_angle;
	}

	// Callback for drive command of car1
	void drive1Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
	{
		if (!use_car1_) return; // NEW (defensive)
		desired_accel1_ = msg->drive.acceleration;
		desired_steer_ang1_ = msg->drive.steering_angle;
	}

	// Publish state of car0
	void state0Publisher()
	{
		state0_pub_->publish(car_state0_);
	}

	// Publish state of car1
	void state1Publisher()
	{
		if (!use_car1_) return; // NEW (defensive)
		state1_pub_->publish(car_state1_);
	}

	void setInput(control_msgs::msg::CarState &state, double desired_accel, double desired_steer_ang, CarParams car_params)
	{

		double dt = 1.0 / simulator_frequency_;
		double steer_diff = desired_steer_ang - state.steer;
		double steer_diff_abs = std::abs(steer_diff);
		double steer_change_max = car_params.steer_vel_max * dt;

		if (steer_diff_abs > steer_change_max)
		{
			state.steer += steer_change_max * (steer_diff / steer_diff_abs);
		}
		else
		{
			state.steer += steer_diff;
		}

		if (state.steer > car_params.steer_max)
		{
			state.steer = car_params.steer_max;
		}
		else if (state.steer < -car_params.steer_max)
		{
			state.steer = -car_params.steer_max;
		}

		double accel_diff = desired_accel - state.accel;
		double accel_diff_abs = std::abs(accel_diff);
		double accel_change_max = car_params.jerk_max * dt;

		if (accel_diff_abs > accel_change_max)
		{
			state.accel += accel_change_max * (accel_diff / accel_diff_abs);
		}
		else
		{
			state.accel += accel_diff;
		}

		if (state.accel > car_params.accel_max)
		{
			state.accel = car_params.accel_max;
		}
		else if (state.accel < -car_params.decel_max)
		{
			state.accel = -car_params.decel_max;
		}

		// Check for NaN values
		if (std::isnan(state.steer) || std::isnan(state.accel))
		{
			RCLCPP_WARN(this->get_logger(), "NaN detected in setInput and will be reset.");
			state.steer = 0.0;
			state.accel = 0.0;
		}
	}

	control_msgs::msg::CarState update_k(const control_msgs::msg::CarState start, double accel, double steer_vel, CarParams p, double dt)
	{
		control_msgs::msg::CarState end;

		// compute first derivatives of state
		double x_dot = start.v * std::cos(start.yaw);
		double y_dot = start.v * std::sin(start.yaw);
		double v_dot = accel;
		double steer_angle_dot = steer_vel;
		double theta_dot = start.v / (p.l_f + p.l_r) * std::tan(start.steer);
		// double theta_double_dot = accel / (p.l_f + p.l_r) * std::tan(start.steer) +
		// 							start.v * steer_vel / ((p.l_f + p.l_r) * std::pow(std::cos(start.steer), 2));
		// double slip_angle_dot = 0;

		// update state
		end.px = start.px + x_dot * dt;
		end.py = start.py + y_dot * dt;
		end.yaw = start.yaw + theta_dot * dt;
		end.v = start.v + v_dot * dt;
		end.steer = start.steer + steer_angle_dot * dt;
		end.omega = 0;		// start.angular_velocity + theta_double_dot * dt;
		end.slip_angle = 0; // start.slip_angle + slip_angle_dot * dt;

		if (end.yaw > M_PI)
			end.yaw -= 2 * M_PI;
		else if (end.yaw < -M_PI)
			end.yaw += 2 * M_PI;

		return end;
	}
	control_msgs::msg::CarState updateStateSingleTrack(control_msgs::msg::CarState &start, CarParams p)
	{
		if (abs(start.v) < 0.1)
		{
			return update_k(start, start.accel, start.steer_vel, p, 1.0 / simulator_frequency_);
		}
		double g = 9.81;
		double h_cg = 0.074;
		double friction_coeff = 0.8;
		double cs_f = 4.718;
		double cs_r = 5.74562;
		double dt = 1.0 / simulator_frequency_;

		double x_dot = start.v * cos(start.yaw + start.slip_angle);
		double y_dot = start.v * sin(start.yaw + start.slip_angle);
		double v_dot = start.accel;
		// double steer_angle_dot = start.steer_vel;
		// double theta_dot = start.omega;123

		double rear_val = g * p.l_r - start.accel * h_cg;
		double front_val = g * p.l_f + start.accel * h_cg;

		// double vel_ratio, first_term;

		// vel_ratio = start.omega / start.v;
		// first_term = friction_coeff / (start.v * (p.l_f + p.l_r));

		double omega_dot =
			(friction_coeff * p.mass / (p.I_z * (p.l_f + p.l_r))) *
			(p.l_f * cs_f * start.steer * (rear_val) + start.slip_angle * (p.l_r * cs_r * (front_val)-p.l_f * cs_f * (rear_val)) -
			 (start.omega / start.v) * (pow(p.l_f, 2) * cs_f * (rear_val) + pow(p.l_r, 2) * cs_r * (front_val)));

		double slip_angle_dot =
			(friction_coeff / (start.v * (p.l_r + p.l_f))) *
				(cs_f * start.steer * rear_val - start.slip_angle * (cs_r * front_val + cs_f * rear_val) +
				 (start.omega / start.v) * (cs_r * p.l_r * front_val - cs_f * p.l_f * rear_val)) -
			start.omega;

		control_msgs::msg::CarState end;
		end.px = start.px + x_dot * dt;
		end.py = start.py + y_dot * dt;
		end.yaw = start.yaw + start.omega * dt;
		end.slip_angle = start.slip_angle + slip_angle_dot * dt;

		end.v = start.v + v_dot * dt;
		end.vx = start.v * cos(start.slip_angle);
		end.vy = start.v * sin(start.slip_angle);
		end.omega = start.omega + omega_dot * dt;

		end.a = start.accel;
		end.ax = start.a * cos(start.slip_angle) - start.v * start.omega * sin(start.slip_angle);
		end.ay = start.a * sin(start.slip_angle) + start.v * start.omega * cos(start.slip_angle);

		end.accel = start.accel;
		end.steer = start.steer;

		if (end.v > p.speed_max)
		{
			end.v = p.speed_max;
		}
		else if (end.v < -p.speed_max)
		{
			end.v = -p.speed_max;
		}

		if (end.yaw > M_PI)
		{
			end.yaw -= 2 * M_PI;
		}
		else if (end.yaw < -M_PI)
		{
			end.yaw += 2 * M_PI;
		}

		while (end.slip_angle > M_PI)
		{
			end.slip_angle -= 2 * M_PI;
		}
		while (end.slip_angle < -M_PI)
		{
			end.slip_angle += 2 * M_PI;
		}

		return end;
	}

	// Update car state
	control_msgs::msg::CarState updateStatePacejka(control_msgs::msg::CarState &start, CarParams car_params)
	{
		if (abs(start.v) < 1.0e-8)
		{
			return update_k(start, start.accel, start.steer_vel, car_params, 1.0 / simulator_frequency_);
		}
		// Implement the update function for car
		control_msgs::msg::CarState end;
		double dt = 1.0 / simulator_frequency_;
		double a_f = -atan2(start.vy + car_params.l_f * start.omega, start.vx) + start.steer;
		double F_fy = car_params.D_f * sin(car_params.C_f * atan(car_params.B_f * a_f));
		double a_r = -atan2(start.vy - car_params.l_r * start.omega, start.vx);
		double F_ry = car_params.D_r * sin(car_params.C_r * atan(car_params.B_r * a_r));

		double x_dot = start.v * cos(start.yaw + start.slip_angle);
		double y_dot = start.v * sin(start.yaw + start.slip_angle);
		double yaw_dot = start.omega;
		double slip_angle_dot = ((F_fy + F_ry) / (car_params.mass * start.v)) - start.omega;
		double v_dot = start.a;
		double omega_dot = (car_params.l_f * F_fy * cos(start.steer) - car_params.l_r * F_ry) / car_params.I_z;

		end.px = start.px + x_dot * dt;
		end.py = start.py + y_dot * dt;
		end.yaw = start.yaw + yaw_dot * dt;
		end.slip_angle = start.slip_angle + slip_angle_dot * dt;

		end.v = start.v + v_dot * dt;
		end.vx = start.v * cos(start.slip_angle);
		end.vy = start.v * sin(start.slip_angle);
		end.omega = start.omega + omega_dot * dt;

		end.a = start.accel;
		end.ax = start.a * cos(start.slip_angle) - start.v * start.omega * sin(start.slip_angle);
		end.ay = start.a * sin(start.slip_angle) + start.v * start.omega * cos(start.slip_angle);

		end.accel = start.accel;
		end.steer = start.steer;

		if (end.v > car_params.speed_max)
		{
			end.v = car_params.speed_max;
		}
		else if (end.v < -car_params.speed_max)
		{
			end.v = -car_params.speed_max;
		}

		if (end.yaw > M_PI)
		{
			end.yaw -= 2 * M_PI;
		}
		else if (end.yaw < -M_PI)
		{
			end.yaw += 2 * M_PI;
		}

		while (end.slip_angle > M_PI)
		{
			end.slip_angle -= 2 * M_PI;
		}
		while (end.slip_angle < -M_PI)
		{
			end.slip_angle += 2 * M_PI;
		}

		if (state_noise_mode_)
		{
			end.px += gen_noise(0.0001);
			end.py += gen_noise(0.0001);
			end.yaw += gen_noise(0.0001);
			end.v += gen_noise(0.0001);
			end.vx += gen_noise(0.0001);
			end.vy += gen_noise(0.0001);
			end.omega += gen_noise(0.0001);
			end.a += gen_noise(0.0001);
			end.ax += gen_noise(0.0001);
			end.ay += gen_noise(0.0001);
			end.accel += gen_noise(0.0001);
			end.steer += gen_noise(0.0001);
			end.slip_angle += gen_noise(0.01);
		}

		return end;
	}

	// Callback for map
	void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
	{
		// Get map parameters
		size_t height = msg->info.height;
		size_t width = msg->info.width;
		double resolution = msg->info.resolution;

		// Convert ROS origin to Pose2D
		Pose2D origin;
		origin.x = msg->info.origin.position.x;
		origin.y = msg->info.origin.position.y;

		// Convert quaternion to Yaw angle
		tf2::Quaternion quat(msg->info.origin.orientation.x,
							 msg->info.origin.orientation.y,
							 msg->info.origin.orientation.z,
							 msg->info.origin.orientation.w);
		tf2::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);
		origin.theta = yaw;

		// Check data size
		if (msg->data.size() != height * width)
		{
			RCLCPP_ERROR(this->get_logger(), "Data size mismatch: expected %zu but got %zu", height * width, msg->data.size());
			return;
		}

		// Convert map to probability values
		std::vector<double> map(msg->data.size(), 0.5); // Initialize with default value of 0.5
		for (size_t i = 0; i < msg->data.size(); i++)
		{
			if (msg->data[i] > 100 || msg->data[i] < 0)
			{
				map[i] = 0.5; // Set as unknown area
			}
			else
			{
				map[i] = msg->data[i] / 100.0; // Convert values from 0-100 to probabilities
			}
		}

		// Pass the map to the scanner
		scan_simulator_.set_map(map, height, width, resolution, origin, map_free_threshold_);

		map_exists_ = true;
	}

	// Callback for center path
	void centerPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
	{
		tf2::Quaternion q(
			msg->poses[1].pose.orientation.x,
			msg->poses[1].pose.orientation.y,
			msg->poses[1].pose.orientation.z,
			msg->poses[1].pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		int last_idx = msg->poses.size() - 3;
		init_car_state0_.px = msg->poses[last_idx].pose.position.x;
		init_car_state0_.py = msg->poses[last_idx].pose.position.y;
		init_car_state0_.yaw = yaw;

		// is_pose_init_ = true;
		receive_start_pose_ = true;
	}

	// Publish scan data
	void pub_scan(const control_msgs::msg::CarState &state,
				  const std::string &scan_frame,
				//   std::vector<float> &scan_data_float,
				  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub,
				  sensor_msgs::msg::LaserScan &scan_msg_data)
	{
		if (!map_exists_)
		{
			return;
		}

		Pose2D scan_pose;
		double scan_distance_to_base_link = 0.12;
		if (scan_noise_mode_) {
			scan_pose.x = state.px + scan_distance_to_base_link * cos(state.yaw) + gen_noise(0.001);
			scan_pose.y = state.py + scan_distance_to_base_link * sin(state.yaw) + gen_noise(0.001);
			scan_pose.theta = state.yaw + gen_noise(0.01);
		} else {
			scan_pose.x = state.px + scan_distance_to_base_link * cos(state.yaw);
			scan_pose.y = state.py + scan_distance_to_base_link * sin(state.yaw);
			scan_pose.theta = state.yaw;
		}

		std::vector<double> scan_data = scan_simulator_.scan(scan_pose);
		const size_t n = scan_data.size();
		const double fov = scan_simulator_.get_field_of_view();
		const double inc = scan_simulator_.get_angle_increment();
		if (scan_msg_data.header.frame_id != scan_frame ||
			scan_msg_data.angle_increment != inc ||
			scan_msg_data.angle_min != -fov/2 ||
			scan_msg_data.angle_max !=  fov/2) {

			scan_msg_data.header.frame_id = scan_frame;
			scan_msg_data.angle_min = -fov / 2;
			scan_msg_data.angle_max =  fov / 2;
			scan_msg_data.angle_increment = inc;
			scan_msg_data.range_max = 10.0;
			scan_msg_data.range_min = 0.1;
			scan_msg_data.time_increment = 0.0;
			scan_msg_data.scan_time = 1.0 / pub_frequency_;
		}

		if (scan_msg_data.ranges.size() != n) {
			scan_msg_data.ranges.resize(n);
		}
		if (scan_msg_data.intensities.size() != n) {
			scan_msg_data.intensities.assign(n, 0.0f);
		}

		std::transform(scan_data.begin(), scan_data.end(),
						scan_msg_data.ranges.begin(),
						[](double d){ return static_cast<float>(d); });

		scan_msg_data.header.stamp = this->get_clock()->now();
		scan_pub->publish(scan_msg_data);
	}

	double gen_noise(double std_dev)
	{
		return n01_(rng_) * std_dev;
	}

	// bool check_collision(const sensor_msgs::msg::LaserScan &scan_data)
	// {
	// 	// scan_coordinates.clear();
	// 	for (size_t i = 0; i < scan_data.ranges.size(); i++)
	// 	{
	// 		float angle = scan_data.angle_min + i * scan_data.angle_increment;
	// 		float x = scan_data.ranges[i] * cos(angle);
	// 		float y = scan_data.ranges[i] * sin(angle);

	// 		if (x > x_min && x < x_max && y > y_min && y < y_max)
	// 		{
	// 			return true;
	// 		}
	// 	}
	// 	return false;
	// }
	bool check_collision(const sensor_msgs::msg::LaserScan& s)
	{
		size_t step = 4;
		if (s.ranges.empty() || s.angle_increment == 0.0f) return false;

		// Angle window that actually sees the box (conservative)
		float a0 = s.angle_min, inc = s.angle_increment;
		float a1 = a0 + (s.ranges.size() - 1) * inc;
		auto A = std::array<float,4>{
			std::atan2(y_min, x_min), std::atan2(y_min, x_max),
			std::atan2(y_max, x_min), std::atan2(y_max, x_max)
		};
		float th_min = std::max(*std::min_element(A.begin(), A.end()), a0);
		float th_max = std::min(*std::max_element(A.begin(), A.end()), a1);
		if (th_min > th_max) return false;

		// Radial window [r_min, r_max] from origin to box (conservative)
		auto clamp = [](float v, float lo, float hi){ return std::max(lo, std::min(v, hi)); };
		float nx = clamp(0.0f, x_min, x_max), ny = clamp(0.0f, y_min, y_max);
		float r_min = std::sqrt(nx*nx + ny*ny);
		float r_max = std::sqrt(std::max({x_min*x_min + y_min*y_min,
										x_min*x_min + y_max*y_max,
										x_max*x_max + y_min*y_min,
										x_max*x_max + y_max*y_max}));

		// Index window
		size_t i0 = (size_t)std::max<int>(0, (int)std::ceil((th_min - a0)/inc));
		size_t i1 = (size_t)std::min<int>(s.ranges.size()-1, (int)std::floor((th_max - a0)/inc));
		if (i0 > i1) return false;

		// Incremental rotation (one sin/cos call per loop step)
		float ang = a0 + i0*inc;
		float c = std::cos(ang),  sn = std::sin(ang);
		float cc = std::cos(inc), ss = std::sin(inc);

		auto rot_next = [&](size_t n){
			while(n--) { float nc = c*cc - sn*ss, ns = sn*cc + c*ss; c = nc; sn = ns; }
		};

		for (size_t i = i0; i <= i1; i += step) {
			float r = s.ranges[i];
			if (std::isfinite(r) && r > 0.0f && r >= r_min && r <= r_max) {
				float x = r * c, y = r * sn;
				if (x > x_min && x < x_max && y > y_min && y < y_max) return true;
			}
			rot_next(step);
		}
		return false;
	}

	void pub_colision(
		const sensor_msgs::msg::LaserScan &scan_data,
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub)
	{
		std_msgs::msg::Bool collision_msg;
		collision_msg.data = check_collision(scan_data);
		collision_pub->publish(collision_msg);
	}

	void pub_odom(
		const control_msgs::msg::CarState &state,
		const std::string &frame_id,
		const std::string &child_frame_id,
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub)
	{
		nav_msgs::msg::Odometry odom_msg;
		odom_msg.header.stamp = this->get_clock()->now();
		// odom_msg.header.frame_id = "odom";
		odom_msg.header.frame_id = frame_id;
		odom_msg.child_frame_id = child_frame_id;
		odom_msg.pose.pose.position.x = state.px;
		odom_msg.pose.pose.position.y = state.py;
		odom_msg.pose.pose.position.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, state.yaw);
		odom_msg.pose.pose.orientation = tf2::toMsg(q);

		odom_msg.twist.twist.linear.x = state.vx;
		odom_msg.twist.twist.linear.y = state.vy;
		odom_msg.twist.twist.linear.z = 0.0;
		odom_msg.twist.twist.angular.x = 0.0;
		odom_msg.twist.twist.angular.y = 0.0;
		odom_msg.twist.twist.angular.z = state.omega;
		odom_pub->publish(odom_msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RacecarSimulator>());
	rclcpp::shutdown();
	return 0;
}


