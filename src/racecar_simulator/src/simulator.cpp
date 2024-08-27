#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "control_msgs/msg/car_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std::chrono_literals; // Use chrono literals for timing
using namespace racecar_simulator;

class RacecarSimulator : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr simulator_timer_;
	rclcpp::TimerBase::SharedPtr pub_timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive0_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive1_sub_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan0_pub_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan1_pub_;
	rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state0_pub_;
	rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state1_pub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

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

	std::string drive_topic0_, state_topic0_, drive_topic1_, state_topic1_, scan_topic0_, scan_topic1_;
	std::string pgm_file_path_, yaml_file_path_;
	double simulator_frequency_, pub_frequency_;
	bool state_noise_mode_ = false;
	bool scan_noise_mode_ = false;

	double desired_speed0_, desired_accel0_, desired_steer_ang0_;
	double desired_speed1_, desired_accel1_, desired_steer_ang1_;
	double scan_fov_, scan_std_dev_;
	int scan_beams_;
	double map_free_threshold_;

	bool map_exists_ = false;

public:
	RacecarSimulator()
		: Node("racecar_simulator")
	{
		// General parameters
		this->declare_parameter("simulator_frequency", 1000.0);
		this->declare_parameter("pub_frequency", 40.0);
		this->declare_parameter("scan_beams", 1080);
		this->declare_parameter("scan_field_of_view", 2.0 * M_PI);
		this->declare_parameter("scan_std_dev", 0.01);
		this->declare_parameter("map_free_threshold", 0.2);
		this->declare_parameter("state_noise_mode", false);
		this->declare_parameter("scan_noise_mode", false);
		this->declare_parameter<std::string>("pgm_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.pgm");
		this->declare_parameter<std::string>("yaml_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.yaml");

		this->get_parameter("simulator_frequency", simulator_frequency_);
		this->get_parameter("pub_frequency", pub_frequency_);
		this->get_parameter("scan_beams", scan_beams_);
		this->get_parameter("scan_field_of_view", scan_fov_);
		this->get_parameter("scan_std_dev", scan_std_dev_);
		this->get_parameter("map_free_threshold", map_free_threshold_);
		this->get_parameter("state_noise_mode", state_noise_mode_);
		this->get_parameter("scan_noise_mode", scan_noise_mode_);
		this->get_parameter("pgm_file_path", pgm_file_path_);
		this->get_parameter("yaml_file_path", yaml_file_path_);

		// Car0 parameters
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
		auto pub_period = std::chrono::duration<double>(1.0 / pub_frequency_);

		// Create publishers and subscribers
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		simulator_timer_ = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(simulator_period),
			std::bind(&RacecarSimulator::simulatorLoop, this));

		pub_timer_ = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(pub_period),
			std::bind(&RacecarSimulator::pubLoop, this));

		init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", 1, std::bind(&RacecarSimulator::car0RvizCallback, this, std::placeholders::_1));

		goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"goal_pose", 1, std::bind(&RacecarSimulator::car1RvizCallback, this, std::placeholders::_1));

		drive0_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic0_, 1, std::bind(&RacecarSimulator::drive0Callback, this, std::placeholders::_1));

		drive1_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic1_, 1, std::bind(&RacecarSimulator::drive1Callback, this, std::placeholders::_1));

		map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
			"map", 1, std::bind(&RacecarSimulator::mapCallback, this, std::placeholders::_1));

		scan0_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic0_, 100);
		scan1_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic1_, 100);
		state0_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic0_, 10);
		state1_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic1_, 10);
		map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

		scan_simulator_ = ScanSimulator2D(scan_beams_, scan_fov_, scan_std_dev_);
	}

	// Simulator loop for updating car states
	void simulatorLoop()
	{
		setInput(car_state0_, desired_accel0_, desired_steer_ang0_, car0_params_);
		setInput(car_state1_, desired_accel1_, desired_steer_ang1_, car1_params_);
		car_state0_ = updateState(car_state0_, car0_params_);
		car_state1_ = updateState(car_state1_, car1_params_);
		setTF();
	}

	// Publisher loop for broadcasting car states
	void pubLoop()
	{
		state0Publisher();
		state1Publisher();
		pub_scan(car_state0_, "laser_model0", scan0_pub_);
		pub_scan(car_state1_, "laser_model1", scan1_pub_);
		pub_map();
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
		// publishTransform("base_link0", "scan0", -0.275, 0.0, 0.0);

		publishTransform("map", "base_link1", car_state1_.px, car_state1_.py, car_state1_.yaw);
		publishTransform("front_left_hinge1", "front_left_wheel1", 0.0, 0.0, car_state1_.steer);
		publishTransform("front_right_hinge1", "front_right_wheel1", 0.0, 0.0, car_state1_.steer);
		// publishTransform("base_link1", "scan1", -0.275, 0.0, 0.0);
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

	// Update car state
	control_msgs::msg::CarState updateState(control_msgs::msg::CarState &start, CarParams car_params)
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

	// Publish scan data
	void pub_scan(const control_msgs::msg::CarState &state,
				  const std::string &scan_frame,
				  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub)
	{
		if (!map_exists_)
		{
			return;
		}

		// Get scan data
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

		// convert to float
		std::vector<float> scan_data_float(scan_data.size());
		for (size_t i = 0; i < scan_data.size(); i++)
		{
			scan_data_float[i] = scan_data[i];
		}
		sensor_msgs::msg::LaserScan scan_msg;
		scan_msg.header.stamp = this->get_clock()->now();
		scan_msg.header.frame_id = scan_frame;
		scan_msg.angle_min = -scan_simulator_.get_field_of_view() / 2;
		scan_msg.angle_max = scan_simulator_.get_field_of_view() / 2;
		scan_msg.angle_increment = scan_simulator_.get_angle_increment();
		scan_msg.range_max = 10.0;
		scan_msg.range_min = 0.1;
		scan_msg.ranges = scan_data_float;
		scan_msg.intensities = std::vector<float>(scan_data.size(), 0.0);
		scan_msg.time_increment = 0.0;
		scan_msg.scan_time = 1.0 / pub_frequency_;
		scan_pub->publish(scan_msg);
	}

	double gen_noise(double std_dev)
	{
		double value = 0.0;
		std::random_device rd;
		std::mt19937 gen(rd());
		std::normal_distribution<double> dist(0.0, std_dev);
		value += dist(gen);
		return value;
	}

	// Function to read the PGM file
	bool read_pgm_file(const std::string &file_path, std::vector<int8_t> &map_data, int &width, int &height)
	{
		std::ifstream file(file_path, std::ios::binary);
		if (!file.is_open())
		{
			std::cerr << "Failed to open PGM file: " << file_path << std::endl;
			return false;
		}

		std::string line;
		std::getline(file, line); // Read PGM format (P5)

		if (line != "P5")
		{
			std::cerr << "Invalid PGM file format: " << line << std::endl;
			return false;
		}

		// Skip comments
		while (std::getline(file, line))
		{
			if (line[0] != '#')
				break;
		}

		std::stringstream ss(line);
		ss >> width >> height;

		std::getline(file, line); // Read max grayscale value

		map_data.resize(width * height);

		file.read(reinterpret_cast<char *>(map_data.data()), map_data.size());

		file.close();
		return true;
	}

	void pub_map()
	{
		// Parse YAML file
		YAML::Node yaml_node = YAML::LoadFile(yaml_file_path_);
		double resolution = yaml_node["resolution"].as<double>();
		std::vector<double> origin = yaml_node["origin"].as<std::vector<double>>();
		double occupied_thresh = yaml_node["occupied_thresh"].as<double>();
		double free_thresh = yaml_node["free_thresh"].as<double>();

		// Read PGM file
		std::vector<int8_t> pgm_data;
		int width, height;
		if (!read_pgm_file(pgm_file_path_, pgm_data, width, height))
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to load the map image!");
			return;
		}

		// Create OccupancyGrid message
		nav_msgs::msg::OccupancyGrid msg;
		msg.header.frame_id = "map";
		msg.info.resolution = resolution;
		msg.info.width = width;
		msg.info.height = height;
		msg.info.origin.position.x = origin[0];
		msg.info.origin.position.y = origin[1];
		msg.info.origin.position.z = origin[2];
		msg.info.origin.orientation.x = 0.0;
		msg.info.origin.orientation.y = 0.0;
		msg.info.origin.orientation.z = 0.0;
		msg.info.origin.orientation.w = 1.0;

		// Convert the PGM data to occupancy values
		msg.data.resize(msg.info.width * msg.info.height);
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				// 반전된 Y값 사용
				int reversed_y = height - 1 - y;
				uint8_t pixel = pgm_data[x + reversed_y * width];
				int index = x + y * width;

				if (pixel == 205)
				{
					msg.data[index] = -1; // Unknown
				}
				else if (pixel > occupied_thresh * 255)
				{
					msg.data[index] = 0; // Free
				}
				else if (pixel < free_thresh * 255)
				{
					msg.data[index] = 100; // Occupied
				}
				else
				{
					msg.data[index] = -1; // Unknown
				}
			}
		}

		// Publish the map message
		msg.header.stamp = this->get_clock()->now();
		map_pub_->publish(msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RacecarSimulator>());
	rclcpp::shutdown();
	return 0;
}
