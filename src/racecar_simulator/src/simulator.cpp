#include <chrono>
#include <functional>
#include <memory>
#include <yaml-cpp/yaml.h>
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
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
class RacecarSimulator : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr simulator_timer_;
	rclcpp::TimerBase::SharedPtr pub_timer_;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive0_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive1_sub_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
	rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state0_pub_;
	rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state1_pub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_path_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision0_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision1_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom0_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom1_pub_;

	control_msgs::msg::CarState car_state0_, car_state1_;

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
	std::string pgm_file_path_, yaml_file_path_, csv_file_path_;
	double simulator_frequency_, pub_frequency_;
	bool state_noise_mode_ = false;

	double desired_speed0_, desired_accel0_, desired_steer_ang0_;
	double desired_speed1_, desired_accel1_, desired_steer_ang1_;

	double map_free_threshold_;

	bool map_exists_ = false;
	nav_msgs::msg::OccupancyGrid original_map_;
	nav_msgs::msg::OccupancyGrid current_map_;
	nav_msgs::msg::Path center_path_;

	bool car0_collision_ = false;
	bool car1_collision_ = false;

	std::vector<std::pair<float, float>> scan_coordinates;
	float x_min = -0.3105;
	float x_max = 0.0705;
	float y_min = -0.1397;
	float y_max = 0.1397;

public:
	RacecarSimulator()
		: Node("racecar_simulator")
	{
		// Load parameters
		// Params params = load_parameters(this);
		// General parameters
		this->declare_parameter("simulator_frequency", 1000.0);
		this->declare_parameter("pub_frequency", 100.0);
		this->declare_parameter("map_free_threshold", 0.2);
		this->declare_parameter("state_noise_mode", false);
		this->declare_parameter<std::string>("pgm_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.pgm");
		this->declare_parameter<std::string>("yaml_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.yaml");
		this->declare_parameter<std::string>("center_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/levinelobby_path.csv");

		this->get_parameter("simulator_frequency", simulator_frequency_);
		this->get_parameter("pub_frequency", pub_frequency_);
		this->get_parameter("map_free_threshold", map_free_threshold_);
		this->get_parameter("state_noise_mode", state_noise_mode_);
		this->get_parameter("pgm_file_path", pgm_file_path_);
		this->get_parameter("yaml_file_path", yaml_file_path_);
		this->get_parameter("center_path", csv_file_path_);

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

		auto qos_reliable_1 = rclcpp::QoS(rclcpp::KeepLast(1))
								  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
								  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

		auto simulator_period = std::chrono::duration<double>(1.0 / simulator_frequency_);
		auto pub_period = std::chrono::duration<double>(1.0 / pub_frequency_);

		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		simulator_timer_ = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(simulator_period),
			std::bind(&RacecarSimulator::simulatorLoop, this));

		pub_timer_ = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(pub_period),
			std::bind(&RacecarSimulator::pubLoop, this));

		init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", qos_reliable_1,
			std::bind(&RacecarSimulator::car0RvizCallback, this, std::placeholders::_1));

		goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"goal_pose", qos_reliable_1,
			std::bind(&RacecarSimulator::car1RvizCallback, this, std::placeholders::_1));

		drive0_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic0_, qos_reliable_1,
			std::bind(&RacecarSimulator::drive0Callback, this, std::placeholders::_1));

		drive1_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic1_, qos_reliable_1,
			std::bind(&RacecarSimulator::drive1Callback, this, std::placeholders::_1));

		state0_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic0_, qos_reliable_1);
		state1_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic1_, qos_reliable_1);

		map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos_reliable_1);
		center_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("center_path", qos_reliable_1);

		collision0_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision0", qos_reliable_1);
		collision1_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision1", qos_reliable_1);

		odom0_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom0", qos_reliable_1);
		odom1_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom1", qos_reliable_1);

		original_map_ = read_map_files(pgm_file_path_, yaml_file_path_);
		current_map_ = original_map_;

		center_path_ = createCenterPathFromCsv(csv_file_path_);
		// Initialize simulator
		RCLCPP_INFO(this->get_logger(), "Racecar simulator initialized");
		RCLCPP_INFO(this->get_logger(), "Simulator frequency: %f Hz", simulator_frequency_);
		RCLCPP_INFO(this->get_logger(), "Publish frequency: %f Hz", pub_frequency_);
		RCLCPP_INFO(this->get_logger(), "vehicle_model0: %d", vehicle_model0_);
		RCLCPP_INFO(this->get_logger(), "vehicle_model1: %d", vehicle_model1_);

		// levinelobby
		car_state0_.px = 3;
		car_state0_.py = 0.3;
		car_state0_.yaw = -0.97;

		car_state1_.px = 3.7;
		car_state1_.py = 1.0;
		car_state1_.yaw = -0.99;

		desired_accel0_ = desired_accel1_ = 0.0;
		desired_steer_ang0_ = desired_steer_ang1_ = 0.0;
	}

	// Simulator loop for updating car states
	void simulatorLoop()
	{
		setInput(car_state0_, desired_accel0_, desired_steer_ang0_, car0_params_);
		setInput(car_state1_, desired_accel1_, desired_steer_ang1_, car1_params_);

		updateState();

		setTF();
	}

	// Publisher loop for broadcasting car states
	void pubLoop()
	{
		current_map_ = original_map_;

		state0Publisher();
		state1Publisher();
		pub_odom(car_state0_, "map", "base_link0", odom0_pub_);
		pub_odom(car_state1_, "map", "base_link1", odom1_pub_);

		pub_map(current_map_);
		pub_center_path(center_path_);
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
		// q.normalize();
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

		publishTransform("map", "base_link1", car_state1_.px, car_state1_.py, car_state1_.yaw);
		publishTransform("front_left_hinge1", "front_left_wheel1", 0.0, 0.0, car_state1_.steer);
		publishTransform("front_right_hinge1", "front_right_wheel1", 0.0, 0.0, car_state1_.steer);
	}

	void updateState()
	{
		car_state0_ = updateStatePacejka(car_state0_, car0_params_);
		car_state1_ = updateStatePacejka(car_state1_, car1_params_);
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

	// 각도 정규화 [-pi, pi]
	static inline double normalizeAngle(double a)
	{
		return std::remainder(a, 2.0 * M_PI);
	}

	// 안전한 나눗셈(분모가 너무 작으면 eps로 대체)
	static inline double safe_div(double num, double den, double eps = 1e-8)
	{
		return num / (std::fabs(den) < eps ? (den >= 0.0 ? eps : -eps) : den);
	}

	control_msgs::msg::CarState updateStatePacejka(control_msgs::msg::CarState &start,
												   const CarParams &car_params)
	{
		const double dt = 1.0 / simulator_frequency_;
		const double eps_v = 1.0e-6; // 속도/분모 안정화용
		const double eps_vx = 1.0e-6;

		// 아주 느릴 때는 기존의 저속 전용 업데이트 사용
		if (std::fabs(start.v) < 1.0e-8)
		{
			return update_k(start, start.accel, start.steer_vel, car_params, dt);
		}

		// 입력(명령)을 현재 가속/조향으로 간주: a_cmd = accel, steer rate = steer_vel
		// 조향 각 적분(존재 시 제한)
		double steer_next = start.steer + start.steer_vel * dt;
		if (std::isfinite(car_params.steer_max) && car_params.steer_max > 0.0)
		{
			steer_next = std::clamp(steer_next, -car_params.steer_max, car_params.steer_max);
		}

		// Pacejka 코너링 강성(전/후) — 사이드슬립 각 계산 시 분모 안정화
		const double vx_safe_f = std::copysign(std::max(std::fabs(start.vx), eps_vx), start.vx);
		const double vx_safe_r = vx_safe_f;

		const double a_f = -std::atan2(start.vy + car_params.l_f * start.omega, vx_safe_f) + steer_next;
		const double a_r = -std::atan2(start.vy - car_params.l_r * start.omega, vx_safe_r);

		const double F_fy = car_params.D_f * std::sin(car_params.C_f * std::atan(car_params.B_f * a_f));
		const double F_ry = car_params.D_r * std::sin(car_params.C_r * std::atan(car_params.B_r * a_r));

		// 운동학(월드 좌표 위치, 요) — 명확히 오일러 전진
		const double x_dot = start.v * std::cos(start.yaw + start.slip_angle);
		const double y_dot = start.v * std::sin(start.yaw + start.slip_angle);
		const double yaw_dot = start.omega;

		// 동역학(바디 좌표 속도/슬립/요속)
		// v_dot은 입력 가속 a(= accel 명령)를 사용
		const double v_dot = start.accel;

		// slip_angle_dot = (ΣFy / (m v)) - ω  (v가 너무 작을 때 폭주 방지)
		const double v_safe = (std::fabs(start.v) < eps_v) ? (start.v >= 0.0 ? eps_v : -eps_v) : start.v;
		const double slip_angle_dot = safe_div((F_fy + F_ry), (car_params.mass * v_safe)) - start.omega;

		// ω 점화: 앞축의 조향 각을 고려(코사인 항)
		const double omega_dot =
			(car_params.l_f * F_fy * std::cos(steer_next) - car_params.l_r * F_ry) / car_params.I_z;

		// 적분
		control_msgs::msg::CarState end = start; // 기본 복사 후 필요한 것만 갱신
		end.px = start.px + x_dot * dt;
		end.py = start.py + y_dot * dt;
		end.yaw = normalizeAngle(start.yaw + yaw_dot * dt);
		end.slip_angle = normalizeAngle(start.slip_angle + slip_angle_dot * dt);

		// 속도 스칼라 v 먼저 갱신 후 제한
		end.v = start.v + v_dot * dt;
		if (car_params.speed_max > 0.0 && std::isfinite(car_params.speed_max))
		{
			end.v = std::clamp(end.v, -car_params.speed_max, car_params.speed_max);
		}

		// 제한된 v와 최신 slip_angle로 바디 좌표 성분 재계산(순서 중요)
		end.vx = end.v * std::cos(end.slip_angle);
		end.vy = end.v * std::sin(end.slip_angle);

		// 요속 적분
		end.omega = start.omega + omega_dot * dt;

		// 바디 좌표 가속도(편의상 vx, vy 시간미분으로 정의)
		// vx = v cosβ, vy = v sinβ → 미분식
		const double vx_dot = v_dot * std::cos(end.slip_angle) - end.v * slip_angle_dot * std::sin(end.slip_angle);
		const double vy_dot = v_dot * std::sin(end.slip_angle) + end.v * slip_angle_dot * std::cos(end.slip_angle);
		end.ax = vx_dot;
		end.ay = vy_dot;

		// 스칼라 a(=long accel)와 명령/조향 상태 업데이트 정리
		end.a = v_dot;			 // 현재 프레임에서의 종가속(= accel 명령)
		end.accel = start.accel; // 입력 유지
		end.steer = steer_next;	 // 적분 반영
		end.steer_vel = start.steer_vel;

		// 최종 안전: slip_angle, yaw는 이미 normalizeAngle로 정규화됨
		return end;
	}

	static bool read_non_comment_line(std::ifstream &f, std::string &out)
	{
		while (std::getline(f, out))
		{
			if (out.empty())
				continue;
			// 앞뒤 공백 제거
			size_t s = out.find_first_not_of(" \t\r");
			size_t e = out.find_last_not_of(" \t\r");
			if (s == std::string::npos)
				continue;
			out = out.substr(s, e - s + 1);
			if (out.empty())
				continue;
			if (out[0] == '#')
				continue;
			return true;
		}
		return false;
	}

	nav_msgs::msg::OccupancyGrid read_map_files(const std::string &pgm_file_path,
												const std::string &yaml_file_path)
	{
		nav_msgs::msg::OccupancyGrid occupancy_grid;

		// --- YAML 파싱 ---
		YAML::Node yaml_node;
		try
		{
			yaml_node = YAML::LoadFile(yaml_file_path);
		}
		catch (const std::exception &e)
		{
			std::cerr << "YAML load error: " << e.what() << std::endl;
			return nav_msgs::msg::OccupancyGrid();
		}

		// 필수 키
		if (!yaml_node["resolution"] || !yaml_node["origin"] ||
			!yaml_node["occupied_thresh"] || !yaml_node["free_thresh"])
		{
			std::cerr << "YAML missing required keys (resolution/origin/occupied_thresh/free_thresh)\n";
			return nav_msgs::msg::OccupancyGrid();
		}

		const double resolution = yaml_node["resolution"].as<double>();
		const auto origin = yaml_node["origin"].as<std::vector<double>>();
		const double occupied_thresh = yaml_node["occupied_thresh"].as<double>();
		const double free_thresh = yaml_node["free_thresh"].as<double>();
		const bool negate = yaml_node["negate"] ? yaml_node["negate"].as<int>() != 0 : false;
		const std::string mode = yaml_node["mode"] ? yaml_node["mode"].as<std::string>() : "trinary";

		if (origin.size() < 3)
		{
			std::cerr << "YAML origin must be [x, y, yaw]\n";
			return nav_msgs::msg::OccupancyGrid();
		}

		std::ifstream file(pgm_file_path, std::ios::binary);
		if (!file.is_open())
		{
			std::cerr << "Failed to open PGM file: " << pgm_file_path << std::endl;
			return nav_msgs::msg::OccupancyGrid();
		}

		std::string line;
		if (!std::getline(file, line) || line != "P5")
		{
			std::cerr << "Invalid PGM magic (expect P5), got: " << line << std::endl;
			return nav_msgs::msg::OccupancyGrid();
		}

		if (!read_non_comment_line(file, line))
		{
			std::cerr << "Missing PGM width/height line\n";
			return nav_msgs::msg::OccupancyGrid();
		}
		int map_width = 0, map_height = 0;
		{
			std::stringstream ss(line);
			ss >> map_width >> map_height;
			if (map_width <= 0 || map_height <= 0)
			{
				std::cerr << "Invalid PGM size\n";
				return nav_msgs::msg::OccupancyGrid();
			}
		}

		if (!read_non_comment_line(file, line))
		{
			std::cerr << "Missing PGM maxval line\n";
			return nav_msgs::msg::OccupancyGrid();
		}
		int maxval = 0;
		{
			std::stringstream ss(line);
			ss >> maxval;
			if (maxval != 255)
			{
				std::cerr << "Unsupported PGM maxval: " << maxval << " (expect 255)\n";
				return nav_msgs::msg::OccupancyGrid();
			}
		}

		std::vector<uint8_t> pgm_data(static_cast<size_t>(map_width) * map_height);
		file.read(reinterpret_cast<char *>(pgm_data.data()), pgm_data.size());
		if (file.gcount() != static_cast<std::streamsize>(pgm_data.size()))
		{
			std::cerr << "PGM data size mismatch\n";
			return nav_msgs::msg::OccupancyGrid();
		}
		file.close();

		// --- OccupancyGrid info ---
		occupancy_grid.info.resolution = resolution;
		occupancy_grid.info.width = static_cast<uint32_t>(map_width);
		occupancy_grid.info.height = static_cast<uint32_t>(map_height);

		occupancy_grid.info.origin.position.x = origin[0];
		occupancy_grid.info.origin.position.y = origin[1];
		occupancy_grid.info.origin.position.z = 0.0;

		// yaw -> quaternion
		const double yaw = origin[2];
		tf2::Quaternion q;
		q.setRPY(0.0, 0.0, yaw);
		occupancy_grid.info.origin.orientation = tf2::toMsg(q);

		occupancy_grid.data.resize(static_cast<size_t>(map_width) * map_height);

		// 미리 임계치(0~255)로 변환
		const int occ_thr = static_cast<int>(occupied_thresh * 255.0 + 0.5);
		const int free_thr = static_cast<int>(free_thresh * 255.0 + 0.5);
		const int denom = std::max(1, free_thr - occ_thr);

		for (int y = 0; y < map_height; ++y)
		{
			const int reversed_y = map_height - 1 - y;
			for (int x = 0; x < map_width; ++x)
			{
				uint8_t p = pgm_data[static_cast<size_t>(x) + static_cast<size_t>(reversed_y) * map_width];
				if (negate)
					p = static_cast<uint8_t>(255 - p);

				int8_t out = -1;

				if (p == 205)
				{
					out = -1;
				}
				else if (p <= occ_thr)
				{
					out = 100;
				}
				else if (p >= free_thr)
				{
					out = 0;
				}
				else
				{
					if (mode == "scale")
					{
						const double ratio = static_cast<double>(free_thr - static_cast<int>(p)) / static_cast<double>(denom);
						int v = static_cast<int>(std::round(std::clamp(ratio, 0.0, 1.0) * 100.0));
						out = static_cast<int8_t>(std::clamp(v, 0, 100));
					}
					else
					{
						out = -1;
					}
				}

				occupancy_grid.data[static_cast<size_t>(x) + static_cast<size_t>(y) * map_width] = out;
			}
		}

		return occupancy_grid;
	}

	void pub_map(const nav_msgs::msg::OccupancyGrid &map)
	{
		nav_msgs::msg::OccupancyGrid msg = map;
		msg.header.stamp = this->get_clock()->now();
		msg.header.frame_id = "map";
		msg.info.map_load_time = msg.header.stamp;

		map_pub_->publish(msg);
	}
	nav_msgs::msg::Path createCenterPathFromCsv(const std::string &csv_file_path)
	{
		nav_msgs::msg::Path path;
		path.header.frame_id = "map";
		path.header.stamp = this->get_clock()->now();

		std::ifstream file(csv_file_path);
		if (!file.is_open())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", csv_file_path.c_str());
			return path;
		}

		auto trim = [](std::string &s)
		{
			s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch)
											{ return !std::isspace(ch); }));
			s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch)
								 { return !std::isspace(ch); })
						.base(),
					s.end());
		};

		std::string line;
		size_t line_no = 0;
		while (std::getline(file, line))
		{
			++line_no;
			trim(line);
			if (line.empty() || line[0] == '#')
				continue;

			// 쉼표 우선(csv), 아니면 공백 분리도 허용
			std::vector<std::string> tokens;
			{
				std::stringstream ss(line);
				std::string item;
				if (line.find(',') != std::string::npos)
				{
					while (std::getline(ss, item, ','))
					{
						trim(item);
						if (!item.empty())
							tokens.push_back(item);
					}
				}
				else
				{
					// 공백 기반 토큰화
					while (ss >> item)
						tokens.push_back(item);
				}
			}

			if (tokens.size() < 2)
			{
				// 첫 줄이 "x,y" 같은 헤더일 수 있음 → 스킵
				RCLCPP_WARN(this->get_logger(), "Invalid line (need at least 2 numbers) at %zu: %s", line_no, line.c_str());
				continue;
			}

			try
			{
				double x = std::stod(tokens[0]);
				double y = std::stod(tokens[1]);

				geometry_msgs::msg::PoseStamped pose;
				pose.header = path.header; // 동일한 frame과 stamp 사용
				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;

				// 회전은 0으로 가정. (필요하면 인접 점으로 yaw 계산해 넣을 수 있음)
				tf2::Quaternion q;
				q.setRPY(0.0, 0.0, 0.0);
				pose.pose.orientation = tf2::toMsg(q);

				path.poses.push_back(pose);
			}
			catch (const std::exception &e)
			{
				// 헤더 문자열 등 double 변환 실패 시 스킵
				RCLCPP_WARN(this->get_logger(), "Failed to parse numbers at line %zu: %s", line_no, line.c_str());
			}
		}

		file.close();
		RCLCPP_INFO(this->get_logger(), "Loaded %zu poses from CSV.", path.poses.size());
		return path;
	}

	void pub_center_path(const nav_msgs::msg::Path &path_in)
	{
		nav_msgs::msg::Path msg = path_in;
		msg.header.stamp = this->get_clock()->now(); // 최신 타임스탬프로 갱신
		msg.header.frame_id = "map";
		center_path_pub_->publish(msg);
	}

	bool check_collision()
	{
		return false;
	}
	void pub_collision(
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub)
	{
		std_msgs::msg::Bool collision_msg;
		collision_msg.data = check_collision();
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
