#include <chrono>
#include <functional>
#include <memory>
// #include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sim_msgs/msg/car_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include "racecar_simulator/scan_simulator_2d.hpp"

using namespace std::chrono_literals;
using namespace racecar_simulator;

class PIDController
{
private:
	double kp_;
	double ki_;
	double kd_;
	double prev_error_;
	double integral_;

public:
	PIDController()
		: kp_(0.0), ki_(0.0), kd_(0.0), prev_error_(0.0), integral_(0.0) {}
	void set_gains(double kp, double ki, double kd)
	{
		kp_ = kp;
		ki_ = ki;
		kd_ = kd;
	}
	double compute(double setpoint, double measured, double dt)
	{
		double error = setpoint - measured;
		integral_ += error * dt;
		double derivative = (error - prev_error_) / dt;
		prev_error_ = error;
		double result = kp_ * error + ki_ * integral_ + kd_ * derivative;
		return std::min(std::max(result, -100.0), 100.0);
	}
};

class RacecarSimulator : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr simulator_timer_;
	rclcpp::TimerBase::SharedPtr odom_timer_;
	rclcpp::TimerBase::SharedPtr imu_timer_;
	rclcpp::TimerBase::SharedPtr scan_timer_;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive0_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive1_sub_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
	rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr fric_sub_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_path_sub_;
	rclcpp::Publisher<sim_msgs::msg::CarState>::SharedPtr state0_pub_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan0_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision0_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom0_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu0_pub_;
	rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc0_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose0_pub_;

	sim_msgs::msg::CarState car_state0_;

	ScanSimulator2D scan_simulator_;
	double map_free_threshold;

	struct CarParams
	{
		double mass, l_r, l_f, I_z;
		double B_xf, C_xf, D_xf, B_xr, C_xr, D_xr;
		double B_yf, C_yf, D_yf, B_yr, C_yr, D_yr;
		double wheel_radius;
		double power_train_inertia, motor_torque_constant;
		double coulomb_friction, viscous_friction;
		double Cd0, Cd1, Cd2;
	};
	CarParams car0_params_;

	double motor_p_, motor_i_, motor_d_;
	double motor_prev_e_, motor_integral_;
	int vehicle_model0_;

	std::string drive_topic0_, collision_topic0_, state_topic0_, scan_topic0_, odom_topic0_, imu_topic0_, vesc_topic0_, pose_topic0_;
	std::string base_frame0_, scan_frame0_;
	std::string pgm_file_path_, yaml_file_path_;
	double simulator_frequency_, odom_frequency_, imu_frequency_, scan_frequency_;
	bool detect_car_mode_ = false;
	bool state_noise_mode_ = false;
	bool scan_noise_mode_ = false;
	double scan_fov_, scan_std_dev_;
	int scan_beams_;
	double map_free_threshold_;
	std::vector<float> scan_data_float0_;
	sensor_msgs::msg::LaserScan scan_msg_data0_;

	// ---- 3DM-CV7 100 Hz 기준 파라미터 ----
	// const double ACC_NOISE_BASE  = 0.00208;      // [m/s^2]
	// const double ACC_NOISE_K     = 2.69e-5;      // heteroscedastic
	const double ACC_NOISE_BASE = 0.2;	  // [m/s^2]
	const double ACC_NOISE_K = 3.0e-1;	  // heteroscedastic
	const double ACC_BIAS_RW = 2.9e-6;	  // [m/s^2 / sqrt(s)]
	const double ACC_TURNON_SIG = 3.9e-4; // [m/s^2]

	// const double GYRO_NOISE_BASE = 0.00029;      // [rad/s]
	// const double GYRO_NOISE_K    = 3.32e-5;      // heteroscedastic
	const double GYRO_NOISE_BASE = 0.03;   // [rad/s]
	const double GYRO_NOISE_K = 4.0e-2;	   // heteroscedastic
	const double GYRO_BIAS_RW = 1.2e-7;	   // [rad/s / sqrt(s)]
	const double GYRO_TURNON_SIG = 7.0e-5; // [rad/s]

	double bias_ax_{0.0};
	double bias_ay_{0.0};
	double bias_r_{0.0};
	bool bias_initialized_{false};

	bool map_exists_ = false;
	nav_msgs::msg::OccupancyGrid original_map_;
	nav_msgs::msg::OccupancyGrid current_map_;

	grid_map::GridMap friction_map_;

	bool car0_collision_ = false;
	bool car1_collision_ = false;

	std::vector<std::pair<float, float>> scan_coordinates;
	float x_min = -0.32;
	float x_max = 0.08;
	float y_min = -0.2;
	float y_max = 0.2;

	bool receive_start_pose_ = false;
	bool is_pose_init_ = false;
	sim_msgs::msg::CarState init_car_state0_;
	std::mt19937 rng_{std::random_device{}()};
	std::normal_distribution<double> n01_{0.0, 1.0};
	PIDController vel_to_iq_pid_;
	PIDController vel_to_accel_pid_;

public:
	RacecarSimulator()
		: Node("racecar_simulator")
	{
		// General parameters
		this->declare_parameter("simulator_frequency", 200.0);
		this->declare_parameter("odom_frequency", 50.0);
		this->declare_parameter("imu_frequency", 100.0);
		this->declare_parameter("scan_frequency", 40.0);
		this->declare_parameter("scan_beams", 1080);
		this->declare_parameter("scan_field_of_view", 4.71238898);
		this->declare_parameter("scan_std_dev", 0.01);
		this->declare_parameter("map_free_threshold", 0.2);
		this->declare_parameter("detect_car_mode", false);
		this->declare_parameter("state_noise_mode", false);
		this->declare_parameter("scan_noise_mode", false);

		this->get_parameter("simulator_frequency", simulator_frequency_);
		this->get_parameter("odom_frequency", odom_frequency_);
		this->get_parameter("imu_frequency", imu_frequency_);
		this->get_parameter("scan_frequency", scan_frequency_);
		this->get_parameter("scan_beams", scan_beams_);
		this->get_parameter("scan_field_of_view", scan_fov_);
		this->get_parameter("scan_std_dev", scan_std_dev_);
		this->get_parameter("map_free_threshold", map_free_threshold_);
		this->get_parameter("detect_car_mode", detect_car_mode_);
		this->get_parameter("state_noise_mode", state_noise_mode_);
		this->get_parameter("scan_noise_mode", scan_noise_mode_);

		// Car0 parameters
		this->declare_parameter("drive_topic0", "ackermann_cmd0");
		this->declare_parameter("state_topic0", "state0");
		this->declare_parameter("scan_topic0", "scan0");
		this->declare_parameter("collision_topic0", "collision0");
		this->declare_parameter("odom_topic0", "odom0");
		this->declare_parameter("imu_topic0", "imu0");
		this->declare_parameter("pose_topic0", "pose0");
		this->declare_parameter("vesc_topic0", "vesc0");
		this->declare_parameter("base_frame0", "base_link0");
		this->declare_parameter("scan_frame0", "laser_model0");
		this->declare_parameter("mass0", 5.1);
		this->declare_parameter("l_r0", 0.115);
		this->declare_parameter("l_f0", 0.345);
		this->declare_parameter("I_z0", 0.46);
		this->declare_parameter("B_xf0", 1.5);
		this->declare_parameter("C_xf0", 1.5);
		this->declare_parameter("D_xf0", 15.0);
		this->declare_parameter("B_xr0", 1.5);
		this->declare_parameter("C_xr0", 1.5);
		this->declare_parameter("D_xr0", 15.0);
		this->declare_parameter("B_yf0", 1.5);
		this->declare_parameter("C_yf0", 1.5);
		this->declare_parameter("D_yf0", 30.0);
		this->declare_parameter("B_yr0", 1.5);
		this->declare_parameter("C_yr0", 1.5);
		this->declare_parameter("D_yr0", 30.0);
		this->declare_parameter("wheel_radius0", 0.05);
		this->declare_parameter("motor_torque_constant0", 0.00273);
		this->declare_parameter("power_train_inertia0", 7.0e-5);
		this->declare_parameter("coulomb_friction0", 3.0e-3);
		this->declare_parameter("viscous_friction0", 4.0e-6);
		this->declare_parameter("Cd0_0", 0.1);
		this->declare_parameter("Cd1_0", 0.01);
		this->declare_parameter("Cd2_0", 0.1);
		this->declare_parameter("motor_p", 15.0);
		this->declare_parameter("motor_i", 0.1);
		this->declare_parameter("motor_d", 0.01);

		this->get_parameter("drive_topic0", drive_topic0_);
		this->get_parameter("state_topic0", state_topic0_);
		this->get_parameter("scan_topic0", scan_topic0_);
		this->get_parameter("collision_topic0", collision_topic0_);
		this->get_parameter("odom_topic0", odom_topic0_);
		this->get_parameter("imu_topic0", imu_topic0_);
		this->get_parameter("pose_topic0", pose_topic0_);
		this->get_parameter("vesc_topic0", vesc_topic0_);
		this->get_parameter("base_frame0", base_frame0_);
		this->get_parameter("scan_frame0", scan_frame0_);
		this->get_parameter("mass0", car0_params_.mass);
		this->get_parameter("l_r0", car0_params_.l_r);
		this->get_parameter("l_f0", car0_params_.l_f);
		this->get_parameter("I_z0", car0_params_.I_z);
		this->get_parameter("B_xf0", car0_params_.B_xf);
		this->get_parameter("C_xf0", car0_params_.C_xf);
		this->get_parameter("D_xf0", car0_params_.D_xf);
		this->get_parameter("B_xr0", car0_params_.B_xr);
		this->get_parameter("C_xr0", car0_params_.C_xr);
		this->get_parameter("D_xr0", car0_params_.D_xr);
		this->get_parameter("B_yf0", car0_params_.B_yf);
		this->get_parameter("C_yf0", car0_params_.C_yf);
		this->get_parameter("D_yf0", car0_params_.D_yf);
		this->get_parameter("B_yr0", car0_params_.B_yr);
		this->get_parameter("C_yr0", car0_params_.C_yr);
		this->get_parameter("D_yr0", car0_params_.D_yr);
		this->get_parameter("wheel_radius0", car0_params_.wheel_radius);
		this->get_parameter("power_train_inertia0", car0_params_.power_train_inertia);
		this->get_parameter("motor_torque_constant0", car0_params_.motor_torque_constant);
		this->get_parameter("coulomb_friction0", car0_params_.coulomb_friction);
		this->get_parameter("viscous_friction0", car0_params_.viscous_friction);
		this->get_parameter("Cd0_0", car0_params_.Cd0);
		this->get_parameter("Cd1_0", car0_params_.Cd1);
		this->get_parameter("Cd2_0", car0_params_.Cd2);
		this->get_parameter("motor_p", motor_p_);
		this->get_parameter("motor_i", motor_i_);
		this->get_parameter("motor_d", motor_d_);

		vel_to_iq_pid_.set_gains(motor_p_, motor_i_, motor_d_);
		vel_to_accel_pid_.set_gains(10.0, 0.0, 0.5);

		// Convert frequencies to durations
		auto simulator_period = std::chrono::duration<double>(1.0 / simulator_frequency_);
		auto odom_period = std::chrono::duration<double>(1.0 / odom_frequency_);
		auto imu_period = std::chrono::duration<double>(1.0 / imu_frequency_);
		auto scan_period = std::chrono::duration<double>(1.0 / scan_frequency_);

		// Create publishers and subscribers
		simulator_timer_ = this->create_wall_timer(
			simulator_period,
			std::bind(&RacecarSimulator::simulatorLoop, this));

		odom_timer_ = this->create_wall_timer(
			odom_period,
			std::bind(&RacecarSimulator::odomLoop, this));

		imu_timer_ = this->create_wall_timer(
			imu_period,
			std::bind(&RacecarSimulator::imuLoop, this));

		scan_timer_ = this->create_wall_timer(
			scan_period,
			std::bind(&RacecarSimulator::scanLoop, this));

		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

		auto r_t_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
		auto r_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
		auto b_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

		init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", r_qos, std::bind(&RacecarSimulator::car0RvizCallback, this, std::placeholders::_1));

		drive0_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic0_, b_qos, std::bind(&RacecarSimulator::drive0Callback, this, std::placeholders::_1));

		map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
			"map", r_t_qos, std::bind(&RacecarSimulator::mapCallback, this, std::placeholders::_1));

		fric_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
			"friction_gridmap", r_t_qos, std::bind(&RacecarSimulator::frictionMapCallback, this, std::placeholders::_1));

		scan0_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic0_, r_qos);
		state0_pub_ = this->create_publisher<sim_msgs::msg::CarState>(state_topic0_, r_qos);
		collision0_pub_ = this->create_publisher<std_msgs::msg::Bool>(collision_topic0_, r_qos);
		odom0_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic0_, r_qos);
		imu0_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic0_, r_qos);
		pose0_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic0_, r_qos);
		vesc0_pub_ = this->create_publisher<vesc_msgs::msg::VescStateStamped>(vesc_topic0_, r_qos);

		scan_simulator_ = ScanSimulator2D(scan_beams_, scan_fov_, scan_std_dev_);
		// Initialize simulator
		RCLCPP_INFO(this->get_logger(), "\nRacecar simulator initialized");
		RCLCPP_INFO(this->get_logger(), "\nSimulator frequency: %f Hz", simulator_frequency_);
		RCLCPP_INFO(this->get_logger(), "\nScan frequency: %f Hz", scan_frequency_);
		RCLCPP_INFO(this->get_logger(), "\nOdom frequency: %f Hz", odom_frequency_);
		RCLCPP_INFO(this->get_logger(), "\nIMU frequency: %f Hz", imu_frequency_);
	}

	// utility functions
	inline double wrapAngle(double a)
	{
		// normalize to [-pi, pi]
		return std::remainder(a, 2.0 * M_PI);
	}

	inline double clamp(double x, double lo, double hi)
	{
		return std::min(std::max(x, lo), hi);
	}
	inline double sign0(double x)
	{
		if (x > 0)
			return 1.0;
		if (x < 0)
			return -1.0;
		return 0.0;
	}

	// Simulator loop for updating car states
	void simulatorLoop()
	{
		if (is_pose_init_ == false)
		{
			car_state0_.px = 0.0;
			car_state0_.py = 0.0;
			car_state0_.yaw = 0.0;
			car_state0_.vx = 0.0;
			car_state0_.vy = 0.0;
			car_state0_.r = 0.0;
			car_state0_.vw = 0.0;
			car_state0_.ax = 0.0;
			car_state0_.ay = 0.0;
			car_state0_.slip_angle = 0.0;
			car_state0_.slip_rate = 0.0;
			car_state0_.accel_cmd = 0.0;
			car_state0_.vel_cmd = 0.0;
			car_state0_.iq = 0.0;
			car_state0_.steer = 0.0;
			car_state0_.steer_vel = 0.0;
			is_pose_init_ = true;
		}

		updateState();
		pub_state();
		pub_colision(scan_msg_data0_, collision0_pub_);
		setTF();
	}
	void odomLoop()
	{
		pub_odom(car_state0_, base_frame0_, odom0_pub_);
		pub_pose(car_state0_, pose0_pub_);
	}
	void imuLoop()
	{
		pub_imu(car_state0_, base_frame0_, imu0_pub_);
	}
	void scanLoop()
	{
		pub_scan(car_state0_, scan_frame0_, scan0_pub_, scan_msg_data0_);
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
		publishTransform("map", base_frame0_, car_state0_.px, car_state0_.py, car_state0_.yaw);
		publishTransform("front_left_hinge0", "front_left_wheel0", 0.0, 0.0, car_state0_.steer);
		publishTransform("front_right_hinge0", "front_right_wheel0", 0.0, 0.0, car_state0_.steer);
	}

	void updateState()
	{
		car_state0_ = updateStatePacejka(car_state0_, car0_params_);
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
		car_state0_.vx = 0.0;
		car_state0_.vy = 0.0;
		car_state0_.r = 0.0;
		car_state0_.vw = 0.0;
		car_state0_.ax = 0.0;
		car_state0_.ay = 0.0;
		car_state0_.slip_angle = 0.0;
		car_state0_.slip_rate = 0.0;
		car_state0_.accel_cmd = 0.0;
		car_state0_.iq = 0.0;
		car_state0_.steer = 0.0;
		car_state0_.steer_vel = 0.0;

		// publishTransform("map", base_frame0_, car_state0_.px, car_state0_.py, car_state0_.yaw);

		RCLCPP_INFO(this->get_logger(), "\nCar0 x: %f, y: %f, yaw: %f", car_state0_.px, car_state0_.py, car_state0_.yaw);
	}

	double getFrictionAt(double x, double y)
	{
		const std::string layer = "friction";

		if (!friction_map_.exists(layer))
		{
			return 1.0;
		}

		const grid_map::Position position(x, y);

		grid_map::Index index;
		if (!friction_map_.getIndex(position, index))
		{
			RCLCPP_WARN(this->get_logger(), "Friction index is out of range!");
			return 1.0;
		}

		const float v = friction_map_.at(layer, index);
		if (!std::isfinite(v))
		{
			RCLCPP_WARN(this->get_logger(), "Friction value is not finite!");
			return 1.0;
		}

		return static_cast<double>(v);
	}

	sim_msgs::msg::CarState update_k(const sim_msgs::msg::CarState &start,
									 const CarParams &p)
	{
		sim_msgs::msg::CarState end = start;

		const double L = p.l_f + p.l_r;
		const double dt = 1.0 / simulator_frequency_;
		// integrate input
		const double accel_cmd = vel_to_accel_pid_.compute(start.vel_cmd, start.vx, dt);
		const double vx = start.vx + accel_cmd * dt;
		const double vx_mid = start.vx + 0.5 * accel_cmd * dt;

		const double x_dot = vx_mid * std::cos(start.yaw);
		const double y_dot = vx_mid * std::sin(start.yaw);
		const double yaw_dot = vx_mid / L * std::tan(start.steer);

		// update state
		end.px = start.px + x_dot * dt;
		end.py = start.py + y_dot * dt;
		end.yaw = wrapAngle(start.yaw + yaw_dot * dt);
		end.vx = vx;
		end.vy = 0.0;
		end.r = yaw_dot;
		end.vw = vx;
		end.mu = getFrictionAt(end.px, end.py);

		return end;
	}

	// Callback for drive command of car0
	void drive0Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
	{
		car_state0_.steer = msg->drive.steering_angle;
		car_state0_.accel_cmd = msg->drive.acceleration;
		car_state0_.vel_cmd = msg->drive.speed;
	}
	// Update car state using Pacejka tire model
	sim_msgs::msg::CarState updateStatePacejka(const sim_msgs::msg::CarState &start, const CarParams &p)
	{
		sim_msgs::msg::CarState end = start;
		const double dt = 1.0 / simulator_frequency_;

		if (start.vx < 0.01)
		{
			return update_k(start, p);
		}

		const double kappa = (start.vw - start.vx) / start.vx;
		const double alpha_f = std::atan2(start.vy + p.l_f * start.r, start.vx) - start.steer;
		const double alpha_r = std::atan2(start.vy - p.l_r * start.r, start.vx);

		double Fx_f = p.D_xf * std::sin(p.C_xf * std::atan(p.B_xf * kappa));
		double Fx_r = p.D_xr * std::sin(p.C_xr * std::atan(p.B_xr * kappa));
		double Fy_f = -p.D_yf * std::sin(p.C_yf * std::atan(p.B_yf * alpha_f));
		double Fy_r = -p.D_yr * std::sin(p.C_yr * std::atan(p.B_yr * alpha_r));

		const double F_drag = p.Cd0 * sign0(start.vx) + p.Cd1 * start.vx + p.Cd2 * start.vx * start.vx;

		// const double iq = vel_to_iq_pid_.compute(start.accel_cmd, start.ax, dt);
		const double iq = vel_to_iq_pid_.compute(start.vel_cmd, start.vw, dt);

		Fx_f *= start.mu;
		Fx_r *= start.mu;
		Fy_f *= start.mu;
		Fy_r *= start.mu;

		// Fx_f *= 1.0;
		// Fx_r *= 1.0;
		// Fy_f *= 1.0;
		// Fy_r *= 1.0;

		const double x_dot = start.vx * std::cos(start.yaw) - start.vy * std::sin(start.yaw);
		const double y_dot = start.vx * std::sin(start.yaw) + start.vy * std::cos(start.yaw);
		const double yaw_dot = start.r;
		const double vx_dot = (Fx_r + Fx_f * std::cos(start.steer) - Fy_f * std::sin(start.steer) - F_drag) / p.mass + start.vy * start.r;
		const double vy_dot = (Fx_f * std::sin(start.steer) + Fy_r + Fy_f * std::cos(start.steer)) / p.mass - start.vx * start.r;
		const double r_dot = ((Fx_f * std::sin(start.steer) + Fy_f * std::cos(start.steer)) * p.l_f - Fy_r * p.l_r) / p.I_z;
		const double vw_dot = (p.wheel_radius / p.power_train_inertia) *
							  (p.motor_torque_constant * iq - p.wheel_radius * (Fx_f + Fx_r) -
							   p.viscous_friction * start.vw - sign0(start.vw) * p.coulomb_friction);

		end.px = start.px + x_dot * dt;
		end.py = start.py + y_dot * dt;
		end.yaw = wrapAngle(start.yaw + yaw_dot * dt);
		end.vx = start.vx + vx_dot * dt;
		end.vy = start.vy + vy_dot * dt;
		end.r = start.r + r_dot * dt;
		end.vw = start.vw + vw_dot * dt;
		end.ax = vx_dot - start.r * start.vy;
		end.ay = vy_dot + start.r * start.vx;
		end.slip_angle = std::atan2(end.vy, end.vx);
		end.slip_rate = kappa;
		end.iq = iq;
		end.mu = getFrictionAt(end.px, end.py);
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

	void frictionMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
	{
		grid_map::GridMapRosConverter::fromMessage(*msg, friction_map_);
	}

	// Publish state of car0
	void pub_state()
	{
		state0_pub_->publish(car_state0_);
		const double iq = car_state0_.iq;
		vesc_msgs::msg::VescStateStamped vesc_msg;
		vesc_msg.state.current_motor = iq;
		vesc0_pub_->publish(vesc_msg);
	}
	// Publish scan data
	void pub_scan(const sim_msgs::msg::CarState &state,
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
		if (scan_noise_mode_)
		{
			scan_pose.x = state.px + scan_distance_to_base_link * cos(state.yaw) + gen_noise(0.001);
			scan_pose.y = state.py + scan_distance_to_base_link * sin(state.yaw) + gen_noise(0.001);
			scan_pose.theta = state.yaw + gen_noise(0.01);
		}
		else
		{
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
			scan_msg_data.angle_min != -fov / 2 ||
			scan_msg_data.angle_max != fov / 2)
		{

			scan_msg_data.header.frame_id = scan_frame;
			scan_msg_data.angle_min = -fov / 2;
			scan_msg_data.angle_max = fov / 2;
			scan_msg_data.angle_increment = inc;
			scan_msg_data.range_max = 10.0;
			scan_msg_data.range_min = 0.1;
			scan_msg_data.time_increment = 0.0;
			scan_msg_data.scan_time = 1.0 / scan_frequency_;
		}

		if (scan_msg_data.ranges.size() != n)
		{
			scan_msg_data.ranges.resize(n);
		}
		if (scan_msg_data.intensities.size() != n)
		{
			scan_msg_data.intensities.assign(n, 0.0f);
		}

		std::transform(scan_data.begin(), scan_data.end(),
					   scan_msg_data.ranges.begin(),
					   [](double d)
					   { return static_cast<float>(d); });

		scan_msg_data.header.stamp = this->get_clock()->now();
		scan_pub->publish(scan_msg_data);
	}

	bool check_collision(const sensor_msgs::msg::LaserScan &s)
	{
		size_t step = 4;
		if (s.ranges.empty() || s.angle_increment == 0.0f)
			return false;

		// Angle window that actually sees the box (conservative)
		float a0 = s.angle_min, inc = s.angle_increment;
		float a1 = a0 + (s.ranges.size() - 1) * inc;
		auto A = std::array<float, 4>{
			std::atan2(y_min, x_min), std::atan2(y_min, x_max),
			std::atan2(y_max, x_min), std::atan2(y_max, x_max)};
		float th_min = std::max(*std::min_element(A.begin(), A.end()), a0);
		float th_max = std::min(*std::max_element(A.begin(), A.end()), a1);
		if (th_min > th_max)
			return false;

		// Radial window [r_min, r_max] from origin to box (conservative)
		auto clamp = [](float v, float lo, float hi)
		{ return std::max(lo, std::min(v, hi)); };
		float nx = clamp(0.0f, x_min, x_max), ny = clamp(0.0f, y_min, y_max);
		float r_min = std::sqrt(nx * nx + ny * ny);
		float r_max = std::sqrt(std::max({x_min * x_min + y_min * y_min,
										  x_min * x_min + y_max * y_max,
										  x_max * x_max + y_min * y_min,
										  x_max * x_max + y_max * y_max}));

		// Index window
		size_t i0 = (size_t)std::max<int>(0, (int)std::ceil((th_min - a0) / inc));
		size_t i1 = (size_t)std::min<int>(s.ranges.size() - 1, (int)std::floor((th_max - a0) / inc));
		if (i0 > i1)
			return false;

		// Incremental rotation (one sin/cos call per loop step)
		float ang = a0 + i0 * inc;
		float c = std::cos(ang), sn = std::sin(ang);
		float cc = std::cos(inc), ss = std::sin(inc);

		auto rot_next = [&](size_t n)
		{
			while (n--)
			{
				float nc = c * cc - sn * ss, ns = sn * cc + c * ss;
				c = nc;
				sn = ns;
			}
		};

		for (size_t i = i0; i <= i1; i += step)
		{
			float r = s.ranges[i];
			if (std::isfinite(r) && r > 0.0f && r >= r_min && r <= r_max)
			{
				float x = r * c, y = r * sn;
				if (x > x_min && x < x_max && y > y_min && y < y_max)
					return true;
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
		const sim_msgs::msg::CarState &state,
		const std::string &frame_id,
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub)
	{
		nav_msgs::msg::Odometry odom_msg;
		odom_msg.header.stamp = this->get_clock()->now();
		// odom_msg.header.frame_id = "odom";
		odom_msg.header.frame_id = frame_id;
		odom_msg.pose.pose.position.x = state.px;
		odom_msg.pose.pose.position.y = state.py;
		odom_msg.pose.pose.position.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, state.yaw);
		odom_msg.pose.pose.orientation = tf2::toMsg(q);

		odom_msg.twist.twist.linear.x = gen_noise(0.01) + state.vw;
		odom_msg.twist.twist.linear.y = 0.0;
		odom_msg.twist.twist.linear.z = 0.0;
		odom_msg.twist.twist.angular.x = 0.0;
		odom_msg.twist.twist.angular.y = 0.0;
		odom_msg.twist.twist.angular.z = state.r;
		odom_pub->publish(odom_msg);
	}
	double gen_noise(double std_dev)
	{
		return n01_(rng_) * std_dev;
	}
	void update_bias_random_walk(double &bias, double sigma_rw, double dt)
	{
		double dw = gen_noise(sigma_rw * std::sqrt(dt));
		bias += dw;
	}
	void init_biases_if_needed()
	{
		if (bias_initialized_)
			return;

		bias_ax_ = gen_noise(ACC_TURNON_SIG);
		bias_ay_ = gen_noise(ACC_TURNON_SIG);
		bias_r_ = gen_noise(GYRO_TURNON_SIG);

		bias_initialized_ = true;
	}

	void imu_noise(sensor_msgs::msg::Imu &imu_msg, double dt)
	{
		if (dt <= 0.0)
		{
			dt = 1.0 / 100.0;
		}

		init_biases_if_needed();

		// 1) bias random walk
		update_bias_random_walk(bias_ax_, ACC_BIAS_RW, dt);
		update_bias_random_walk(bias_ay_, ACC_BIAS_RW, dt);
		update_bias_random_walk(bias_r_, GYRO_BIAS_RW, dt);

		// 2) ax, ay
		{
			const double ax_true = imu_msg.linear_acceleration.x;
			const double ay_true = imu_msg.linear_acceleration.y;

			const double sigma_ax = ACC_NOISE_BASE + ACC_NOISE_K * std::abs(ax_true);
			const double sigma_ay = ACC_NOISE_BASE + ACC_NOISE_K * std::abs(ay_true);

			imu_msg.linear_acceleration.x = ax_true + bias_ax_ + gen_noise(sigma_ax);
			imu_msg.linear_acceleration.y = ay_true + bias_ay_ + gen_noise(sigma_ay);
		}

		// 3) r (yaw rate)
		{
			const double r_true = imu_msg.angular_velocity.z;
			const double sigma_r = GYRO_NOISE_BASE + GYRO_NOISE_K * std::abs(r_true);

			imu_msg.angular_velocity.z = r_true + bias_r_ + gen_noise(sigma_r);
		}
	}

	void pub_imu(
		const sim_msgs::msg::CarState &state,
		const std::string &frame_id,
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub)
	{
		sensor_msgs::msg::Imu imu_msg;
		imu_msg.header.stamp = this->get_clock()->now();
		imu_msg.header.frame_id = frame_id;

		tf2::Quaternion q;
		q.setRPY(0, 0, state.yaw);
		imu_msg.orientation = tf2::toMsg(q);

		imu_msg.angular_velocity.x = 0.0;
		imu_msg.angular_velocity.y = 0.0;
		// imu_msg.angular_velocity.z = state.r;
		imu_msg.angular_velocity.z = gen_noise(0.01) + state.r;

		// imu_msg.linear_acceleration.x = state.ax;
		// imu_msg.linear_acceleration.y = state.ay;
		imu_msg.linear_acceleration.x = gen_noise(0.5) + state.ax;
		imu_msg.linear_acceleration.y = gen_noise(0.5) + state.ay;
		imu_msg.linear_acceleration.z = 0.0;

		// double dt = 1.0 / imu_frequency_;
		// imu_noise(imu_msg, dt);

		imu_pub->publish(imu_msg);
	}

	void pub_pose(
		const sim_msgs::msg::CarState &state,
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub)
	{
		geometry_msgs::msg::PoseStamped pose_msg;
		pose_msg.header.stamp = this->get_clock()->now();
		pose_msg.header.frame_id = "map";
		pose_msg.pose.position.x = state.px;
		pose_msg.pose.position.y = state.py;
		pose_msg.pose.position.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, state.yaw);
		pose_msg.pose.orientation = tf2::toMsg(q);

		pose_pub->publish(pose_msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RacecarSimulator>());
	rclcpp::shutdown();
	return 0;
}
