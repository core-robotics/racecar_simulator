#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "control_msgs/msg/car_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace utils
{
	inline double normalizeAngle(double a)
	{
		return std::remainder(a, 2.0 * M_PI);
	}

	inline double safeDiv(double num, double den, double eps = 1e-8)
	{
		return num / (std::fabs(den) < eps ? (den >= 0.0 ? eps : -eps) : den);
	}

	inline bool isFinite(double x) { return std::isfinite(x); }

	inline bool hasNaN(const geometry_msgs::msg::Transform &t)
	{
		return std::isnan(t.translation.x) || std::isnan(t.translation.y) || std::isnan(t.translation.z) ||
			   std::isnan(t.rotation.x) || std::isnan(t.rotation.y) || std::isnan(t.rotation.z) || std::isnan(t.rotation.w);
	}
} // namespace utils

class RacecarSimulator : public rclcpp::Node
{
public:
	RacecarSimulator()
		: Node("racecar_simulator")
	{
		declareAndGetParameters();
		initROS();
		loadWorld();
		initCars();

		RCLCPP_INFO(get_logger(), "Racecar simulator initialized");
		RCLCPP_INFO(get_logger(), "Simulator frequency: %.2f Hz", simulator_frequency_);
		RCLCPP_INFO(get_logger(), "Publish frequency: %.2f Hz", pub_frequency_);
		RCLCPP_INFO(get_logger(), "vehicle_model0: %d", vehicle_model0_);
		RCLCPP_INFO(get_logger(), "vehicle_model1: %d", vehicle_model1_);
	}

private:
	// ===== Types =====
	struct CarParams
	{
		double mass{}, l_r{}, l_f{}, I_z{};
		double B_f{}, C_f{}, D_f{}, B_r{}, C_r{}, D_r{};
		double steer_max{}, steer_vel_max{};
		double speed_max{}, accel_max{}, decel_max{}, jerk_max{};
	};

	// ===== Parameters & State =====
	// Timers
	rclcpp::TimerBase::SharedPtr simulator_timer_;
	rclcpp::TimerBase::SharedPtr pub_timer_;

	// TF
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	// Subs
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive0_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive1_sub_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; // reserved (not used yet)

	// Pubs
	rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state0_pub_;
	rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr state1_pub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_path_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision0_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision1_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom0_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom1_pub_;

	// Vehicles
	control_msgs::msg::CarState car_state0_{};
	control_msgs::msg::CarState car_state1_{};
	CarParams car0_params_{};
	CarParams car1_params_{};

	// Global params
	double simulator_frequency_{};
	double pub_frequency_{};
	bool state_noise_mode_{}; // currently unused, kept for compat

	// Topics/ids
	int vehicle_model0_{};
	int vehicle_model1_{};
	std::string drive_topic0_{"ackermann_cmd0"};
	std::string state_topic0_{"state0"};
	std::string scan_topic0_{"scan0"};
	std::string drive_topic1_{"ackermann_cmd1"};
	std::string state_topic1_{"state1"};
	std::string scan_topic1_{"scan1"};

	// Files
	std::string pgm_file_path_{};
	std::string yaml_file_path_{};
	std::string csv_file_path_{};

	// World
	nav_msgs::msg::OccupancyGrid original_map_{};
	nav_msgs::msg::OccupancyGrid current_map_{};
	nav_msgs::msg::Path center_path_{};

	// Inputs (desired)
	double desired_accel0_{};
	double desired_steer_ang0_{};
	double desired_accel1_{};
	double desired_steer_ang1_{};

	// ===== Initialization =====
	void declareAndGetParameters()
	{
		// General
		this->declare_parameter("simulator_frequency", 1000.0);
		this->declare_parameter("pub_frequency", 100.0);
		this->declare_parameter("state_noise_mode", false);
		this->declare_parameter<std::string>("pgm_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.pgm");
		this->declare_parameter<std::string>("yaml_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.yaml");
		this->declare_parameter<std::string>("center_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/levinelobby_path.csv");

		this->get_parameter("simulator_frequency", simulator_frequency_);
		this->get_parameter("pub_frequency", pub_frequency_);
		this->get_parameter("state_noise_mode", state_noise_mode_);
		this->get_parameter("pgm_file_path", pgm_file_path_);
		this->get_parameter("yaml_file_path", yaml_file_path_);
		this->get_parameter("center_path", csv_file_path_);

		// Car0
		this->declare_parameter("vehicle_model0", 1);
		this->declare_parameter("drive_topic0", drive_topic0_);
		this->declare_parameter("state_topic0", state_topic0_);
		this->declare_parameter("scan_topic0", scan_topic0_);
		declareCarParams("0", car0_params_, /*defaults=*/{3.5, 0.17145, 0.17145, 0.04712, 1.5, 1.5, 30.0, 1.5, 1.5, 30.0, 4.0, 4.0, 10.0, 40.0, 40.0, 100.0});

		// Car1
		this->declare_parameter("vehicle_model1", 1);
		this->declare_parameter("drive_topic1", drive_topic1_);
		this->declare_parameter("state_topic1", state_topic1_);
		this->declare_parameter("scan_topic1", scan_topic1_);
		declareCarParams("1", car1_params_, /*defaults=*/{3.5, 0.17145, 0.17145, 0.04712, 1.5, 1.5, 30.0, 1.5, 1.5, 30.0, 0.4, 0.041, 10.0, 4.0, 4.0, 1.0});

		this->get_parameter("vehicle_model0", vehicle_model0_);
		this->get_parameter("vehicle_model1", vehicle_model1_);
		this->get_parameter("drive_topic0", drive_topic0_);
		this->get_parameter("state_topic0", state_topic0_);
		this->get_parameter("scan_topic0", scan_topic0_);
		this->get_parameter("drive_topic1", drive_topic1_);
		this->get_parameter("state_topic1", state_topic1_);
		this->get_parameter("scan_topic1", scan_topic1_);
	}

	void declareCarParams(const std::string &suffix, CarParams &out, const CarParams &defaults)
	{
		this->declare_parameter("mass" + suffix, defaults.mass);
		this->declare_parameter("l_r" + suffix, defaults.l_r);
		this->declare_parameter("l_f" + suffix, defaults.l_f);
		this->declare_parameter("I_z" + suffix, defaults.I_z);
		this->declare_parameter("B_f" + suffix, defaults.B_f);
		this->declare_parameter("C_f" + suffix, defaults.C_f);
		this->declare_parameter("D_f" + suffix, defaults.D_f);
		this->declare_parameter("B_r" + suffix, defaults.B_r);
		this->declare_parameter("C_r" + suffix, defaults.C_r);
		this->declare_parameter("D_r" + suffix, defaults.D_r);
		this->declare_parameter("steer_max" + suffix, defaults.steer_max);
		this->declare_parameter("steer_vel_max" + suffix, defaults.steer_vel_max);
		this->declare_parameter("speed_max" + suffix, defaults.speed_max);
		this->declare_parameter("accel_max" + suffix, defaults.accel_max);
		this->declare_parameter("decel_max" + suffix, defaults.decel_max);
		this->declare_parameter("jerk_max" + suffix, defaults.jerk_max);

		this->get_parameter("mass" + suffix, out.mass);
		this->get_parameter("l_r" + suffix, out.l_r);
		this->get_parameter("l_f" + suffix, out.l_f);
		this->get_parameter("I_z" + suffix, out.I_z);
		this->get_parameter("B_f" + suffix, out.B_f);
		this->get_parameter("C_f" + suffix, out.C_f);
		this->get_parameter("D_f" + suffix, out.D_f);
		this->get_parameter("B_r" + suffix, out.B_r);
		this->get_parameter("C_r" + suffix, out.C_r);
		this->get_parameter("D_r" + suffix, out.D_r);
		this->get_parameter("steer_max" + suffix, out.steer_max);
		this->get_parameter("steer_vel_max" + suffix, out.steer_vel_max);
		this->get_parameter("speed_max" + suffix, out.speed_max);
		this->get_parameter("accel_max" + suffix, out.accel_max);
		this->get_parameter("decel_max" + suffix, out.decel_max);
		this->get_parameter("jerk_max" + suffix, out.jerk_max);
	}

	void initROS()
	{
		auto qos_reliable_1 = rclcpp::QoS(rclcpp::KeepLast(1))
								  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
								  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		const auto simulator_period = std::chrono::duration<double>(1.0 / simulator_frequency_);
		const auto pub_period = std::chrono::duration<double>(1.0 / pub_frequency_);

		simulator_timer_ = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(simulator_period),
			std::bind(&RacecarSimulator::simulatorLoop, this));

		pub_timer_ = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(pub_period),
			std::bind(&RacecarSimulator::pubLoop, this));

		// Subscriptions
		init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", qos_reliable_1, std::bind(&RacecarSimulator::car0RvizCallback, this, std::placeholders::_1));

		goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"goal_pose", qos_reliable_1, std::bind(&RacecarSimulator::car1RvizCallback, this, std::placeholders::_1));

		drive0_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic0_, qos_reliable_1, std::bind(&RacecarSimulator::drive0Callback, this, std::placeholders::_1));

		drive1_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic1_, qos_reliable_1, std::bind(&RacecarSimulator::drive1Callback, this, std::placeholders::_1));

		// Publishers
		state0_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic0_, qos_reliable_1);
		state1_pub_ = this->create_publisher<control_msgs::msg::CarState>(state_topic1_, qos_reliable_1);
		map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos_reliable_1);
		center_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("center_path", qos_reliable_1);
		collision0_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision0", qos_reliable_1);
		collision1_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision1", qos_reliable_1);
		odom0_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom0", qos_reliable_1);
		odom1_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom1", qos_reliable_1);
	}

	void loadWorld()
	{
		original_map_ = readMapFiles(pgm_file_path_, yaml_file_path_);
		current_map_ = original_map_;
		center_path_ = createCenterPathFromCsv(csv_file_path_);
	}

	void initCars()
	{
		// Initial pose (levinelobby defaults)
		car_state0_.px = 3.0;
		car_state0_.py = 0.3;
		car_state0_.yaw = -0.97;
		car_state1_.px = 3.7;
		car_state1_.py = 1.0;
		car_state1_.yaw = -0.99;

		desired_accel0_ = desired_accel1_ = 0.0;
		desired_steer_ang0_ = desired_steer_ang1_ = 0.0;
	}

	// ===== Timed Loops =====
	void simulatorLoop()
	{
		setInput(car_state0_, desired_accel0_, desired_steer_ang0_, car0_params_);
		setInput(car_state1_, desired_accel1_, desired_steer_ang1_, car1_params_);

		car_state0_ = updateStatePacejka(car_state0_, car0_params_);
		car_state1_ = updateStatePacejka(car_state1_, car1_params_);

		publishTF("map", "base_link0", car_state0_.px, car_state0_.py, car_state0_.yaw);
		publishTF("front_left_hinge0", "front_left_wheel0", 0.0, 0.0, car_state0_.steer);
		publishTF("front_right_hinge0", "front_right_wheel0", 0.0, 0.0, car_state0_.steer);

		publishTF("map", "base_link1", car_state1_.px, car_state1_.py, car_state1_.yaw);
		publishTF("front_left_hinge1", "front_left_wheel1", 0.0, 0.0, car_state1_.steer);
		publishTF("front_right_hinge1", "front_right_wheel1", 0.0, 0.0, car_state1_.steer);
	}

	void pubLoop()
	{
		current_map_ = original_map_;

		state0_pub_->publish(car_state0_);
		state1_pub_->publish(car_state1_);

		publishOdom(car_state0_, "map", "base_link0", odom0_pub_);
		publishOdom(car_state1_, "map", "base_link1", odom1_pub_);

		publishMap(current_map_);
		publishCenterPath(center_path_);

		publishCollision(collision0_pub_);
		publishCollision(collision1_pub_);
	}

	// ===== Callbacks =====
	void car0RvizCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
		tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
						  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		resetState(car_state0_, msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
		desired_accel0_ = 0.0;
		desired_steer_ang0_ = 0.0;
		publishTF("map", "base_link0", car_state0_.px, car_state0_.py, car_state0_.yaw);
		RCLCPP_INFO(get_logger(), "Car0 x: %.3f, y: %.3f, yaw: %.3f", car_state0_.px, car_state0_.py, car_state0_.yaw);
	}

	void car1RvizCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
						  msg->pose.orientation.z, msg->pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		resetState(car_state1_, msg->pose.position.x, msg->pose.position.y, yaw);
		desired_accel1_ = 0.0;
		desired_steer_ang1_ = 0.0;
		publishTF("map", "base_link1", car_state1_.px, car_state1_.py, car_state1_.yaw);
		RCLCPP_INFO(get_logger(), "Car1 x: %.3f, y: %.3f, yaw: %.3f", car_state1_.px, car_state1_.py, car_state1_.yaw);
	}

	void drive0Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
	{
		desired_accel0_ = msg->drive.acceleration;
		desired_steer_ang0_ = msg->drive.steering_angle;
	}

	void drive1Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
	{
		desired_accel1_ = msg->drive.acceleration;
		desired_steer_ang1_ = msg->drive.steering_angle;
	}

	// ===== Helpers =====
	static void resetState(control_msgs::msg::CarState &s, double x, double y, double yaw)
	{
		s.px = x;
		s.py = y;
		s.yaw = yaw;
		s.v = s.a = s.accel = 0.0;
		s.steer = s.steer_vel = 0.0;
		s.vx = s.vy = s.omega = 0.0;
		s.ax = s.ay = s.slip_angle = s.jerk = 0.0;
		s.steer = s.steer_vel = 0.0;
	}

	void publishTF(const std::string &frame_id, const std::string &child_frame_id,
				   double x, double y, double yaw)
	{
		geometry_msgs::msg::TransformStamped t;
		t.header.stamp = get_clock()->now();
		t.header.frame_id = frame_id;
		t.child_frame_id = child_frame_id;
		t.transform.translation.x = x;
		t.transform.translation.y = y;
		t.transform.translation.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, yaw);
		t.transform.rotation = tf2::toMsg(q);

		if (utils::hasNaN(t.transform))
		{
			RCLCPP_WARN(get_logger(), "Transformation contains NaN values and will be ignored.");
			return;
		}
		tf_broadcaster_->sendTransform(t);
	}

	void publishOdom(const control_msgs::msg::CarState &state,
					 const std::string &frame_id,
					 const std::string &child_frame_id,
					 const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pub)
	{
		nav_msgs::msg::Odometry odom;
		odom.header.stamp = get_clock()->now();
		odom.header.frame_id = frame_id;
		odom.child_frame_id = child_frame_id;

		odom.pose.pose.position.x = state.px;
		odom.pose.pose.position.y = state.py;
		odom.pose.pose.position.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, state.yaw);
		odom.pose.pose.orientation = tf2::toMsg(q);

		odom.twist.twist.linear.x = state.vx;
		odom.twist.twist.linear.y = state.vy;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.z = state.omega;

		pub->publish(odom);
	}

	void publishMap(const nav_msgs::msg::OccupancyGrid &map)
	{
		auto msg = map;
		msg.header.stamp = get_clock()->now();
		msg.header.frame_id = "map";
		msg.info.map_load_time = msg.header.stamp;
		map_pub_->publish(msg);
	}

	void publishCenterPath(const nav_msgs::msg::Path &path_in)
	{
		auto msg = path_in;
		msg.header.stamp = get_clock()->now();
		msg.header.frame_id = "map";
		center_path_pub_->publish(msg);
	}

	static bool checkCollision() { return false; }

	void publishCollision(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub)
	{
		std_msgs::msg::Bool msg;
		msg.data = checkCollision();
		pub->publish(msg);
	}

	// ===== Vehicle Input / Dynamics =====
	void setInput(control_msgs::msg::CarState &state,
				  double desired_accel, double desired_steer_ang,
				  const CarParams &p)
	{
		const double dt = 1.0 / simulator_frequency_;

		// Steering rate limit
		const double steer_diff = desired_steer_ang - state.steer;
		const double steer_change_max = p.steer_vel_max * dt;
		if (std::fabs(steer_diff) > steer_change_max)
			state.steer += steer_change_max * (steer_diff / std::fabs(steer_diff));
		else
			state.steer += steer_diff;

		if (utils::isFinite(p.steer_max) && p.steer_max > 0.0)
			state.steer = std::clamp(state.steer, -p.steer_max, p.steer_max);

		// Longitudinal jerk limit (applied to commanded accel state.accel)
		const double accel_diff = desired_accel - state.accel;
		const double accel_change_max = p.jerk_max * dt;
		if (std::fabs(accel_diff) > accel_change_max)
			state.accel += accel_change_max * (accel_diff / std::fabs(accel_diff));
		else
			state.accel += accel_diff;

		state.accel = std::clamp(state.accel, -p.decel_max, p.accel_max);

		if (std::isnan(state.steer) || std::isnan(state.accel))
		{
			RCLCPP_WARN(get_logger(), "NaN detected in setInput and will be reset.");
			state.steer = 0.0;
			state.accel = 0.0;
		}
	}

	control_msgs::msg::CarState updateKinematic(const control_msgs::msg::CarState &start,
												double accel, double steer_vel,
												const CarParams &p, double dt)
	{
		control_msgs::msg::CarState end{};

		const double x_dot = start.v * std::cos(start.yaw);
		const double y_dot = start.v * std::sin(start.yaw);
		const double v_dot = accel;
		const double theta_dot = start.v / (p.l_f + p.l_r) * std::tan(start.steer);

		end.px = start.px + x_dot * dt;
		end.py = start.py + y_dot * dt;
		end.yaw = utils::normalizeAngle(start.yaw + theta_dot * dt);
		end.v = start.v + v_dot * dt;
		end.steer = start.steer + steer_vel * dt;
		end.omega = 0.0;
		end.slip_angle = 0.0;
		return end;
	}

	control_msgs::msg::CarState updateStatePacejka(const control_msgs::msg::CarState &start,
												   const CarParams &p)
	{
		const double dt = 1.0 / simulator_frequency_;
		const double eps_v = 1e-6;
		const double eps_vx = 1e-6;

		if (std::fabs(start.v) < 1e-8)
			return updateKinematic(start, start.accel, start.steer_vel, p, dt);

		// Integrate steering (already rate-limited in setInput, but keep consistent here)
		double steer_next = start.steer + start.steer_vel * dt;
		if (utils::isFinite(p.steer_max) && p.steer_max > 0.0)
			steer_next = std::clamp(steer_next, -p.steer_max, p.steer_max);

		// Slip angles (stabilized denominators)
		const double vx_safe_f = std::copysign(std::max(std::fabs(start.vx), eps_vx), start.vx);
		const double vx_safe_r = vx_safe_f;

		const double a_f = -std::atan2(start.vy + p.l_f * start.omega, vx_safe_f) + steer_next;
		const double a_r = -std::atan2(start.vy - p.l_r * start.omega, vx_safe_r);

		const double F_fy = p.D_f * std::sin(p.C_f * std::atan(p.B_f * a_f));
		const double F_ry = p.D_r * std::sin(p.C_r * std::atan(p.B_r * a_r));

		// Kinematics in world frame
		const double x_dot = start.v * std::cos(start.yaw + start.slip_angle);
		const double y_dot = start.v * std::sin(start.yaw + start.slip_angle);
		const double yaw_dot = start.omega;

		// Dynamics
		const double v_dot = start.accel;
		const double v_safe = (std::fabs(start.v) < eps_v) ? (start.v >= 0.0 ? eps_v : -eps_v) : start.v;
		const double slip_angle_dot = utils::safeDiv((F_fy + F_ry), (p.mass * v_safe)) - start.omega;
		const double omega_dot = (p.l_f * F_fy * std::cos(steer_next) - p.l_r * F_ry) / p.I_z;

		control_msgs::msg::CarState end = start;
		end.px = start.px + x_dot * dt;
		end.py = start.py + y_dot * dt;
		end.yaw = utils::normalizeAngle(start.yaw + yaw_dot * dt);
		end.slip_angle = utils::normalizeAngle(start.slip_angle + slip_angle_dot * dt);

		end.v = start.v + v_dot * dt;
		if (utils::isFinite(p.speed_max) && p.speed_max > 0.0)
			end.v = std::clamp(end.v, -p.speed_max, p.speed_max);

		end.vx = end.v * std::cos(end.slip_angle);
		end.vy = end.v * std::sin(end.slip_angle);
		end.omega = start.omega + omega_dot * dt;

		const double vx_dot = v_dot * std::cos(end.slip_angle) - end.v * slip_angle_dot * std::sin(end.slip_angle);
		const double vy_dot = v_dot * std::sin(end.slip_angle) + end.v * slip_angle_dot * std::cos(end.slip_angle);
		end.ax = vx_dot;
		end.ay = vy_dot;

		end.a = v_dot;
		end.accel = start.accel;
		end.steer = steer_next;
		end.steer_vel = start.steer_vel;

		return end;
	}

	// ===== Map & Path Loading =====
	static bool readNonCommentLine(std::ifstream &ifs, std::string &out)
	{
		while (std::getline(ifs, out))
		{
			if (!out.empty() && out[0] != '#')
			{
				// Trim leading whitespace
				size_t s = out.find_first_not_of(" \t\r");
				if (s == std::string::npos)
				{
					continue;
				}
				// Trim trailing whitespace
				size_t e = out.find_last_not_of(" \t\r");
				out = out.substr(s, e - s + 1);
				return true;
			}
		}
		return false;
	}

	nav_msgs::msg::OccupancyGrid readMapFiles(const std::string &pgm_file_path,
											  const std::string &yaml_file_path)
	{
		nav_msgs::msg::OccupancyGrid grid;

		YAML::Node yaml;
		try
		{
			yaml = YAML::LoadFile(yaml_file_path);
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(get_logger(), "YAML load error: %s", e.what());
			return grid;
		}

		if (!yaml["resolution"] || !yaml["origin"] || !yaml["occupied_thresh"] || !yaml["free_thresh"])
		{
			RCLCPP_ERROR(get_logger(), "YAML missing required keys (resolution/origin/occupied_thresh/free_thresh)");
			return grid;
		}

		const double resolution = yaml["resolution"].as<double>();
		const auto origin = yaml["origin"].as<std::vector<double>>();
		const double occupied_thresh = yaml["occupied_thresh"].as<double>();
		const double free_thresh = yaml["free_thresh"].as<double>();
		const bool negate = yaml["negate"] ? yaml["negate"].as<int>() != 0 : false;
		const std::string mode = yaml["mode"] ? yaml["mode"].as<std::string>() : "trinary";

		if (origin.size() < 3)
		{
			RCLCPP_ERROR(get_logger(), "YAML origin must be [x, y, yaw]");
			return grid;
		}

		std::ifstream file(pgm_file_path, std::ios::binary);
		if (!file.is_open())
		{
			RCLCPP_ERROR(get_logger(), "Failed to open PGM file: %s", pgm_file_path.c_str());
			return grid;
		}

		std::string line;
		if (!std::getline(file, line) || line != "P5")
		{
			RCLCPP_ERROR(get_logger(), "Invalid PGM magic (expect P5), got: %s", line.c_str());
			return grid;
		}

		if (!readNonCommentLine(file, line))
		{
			RCLCPP_ERROR(get_logger(), "Missing PGM width/height line");
			return grid;
		}

		int map_width = 0, map_height = 0;
		{
			std::stringstream ss(line);
			ss >> map_width >> map_height;
			if (map_width <= 0 || map_height <= 0)
			{
				RCLCPP_ERROR(get_logger(), "Invalid PGM size");
				return grid;
			}
		}

		if (!readNonCommentLine(file, line))
		{
			RCLCPP_ERROR(get_logger(), "Missing PGM maxval line");
			return grid;
		}

		int maxval = 0;
		{
			std::stringstream ss(line);
			ss >> maxval;
		}
		if (maxval != 255)
		{
			RCLCPP_ERROR(get_logger(), "Unsupported PGM maxval: %d (expect 255)", maxval);
			return grid;
		}

		std::vector<uint8_t> pgm_data(static_cast<size_t>(map_width) * map_height);
		file.read(reinterpret_cast<char *>(pgm_data.data()), pgm_data.size());
		if (file.gcount() != static_cast<std::streamsize>(pgm_data.size()))
		{
			RCLCPP_ERROR(get_logger(), "PGM data size mismatch");
			return grid;
		}
		file.close();

		// Fill grid info
		grid.info.resolution = resolution;
		grid.info.width = static_cast<uint32_t>(map_width);
		grid.info.height = static_cast<uint32_t>(map_height);
		grid.info.origin.position.x = origin[0];
		grid.info.origin.position.y = origin[1];
		grid.info.origin.position.z = 0.0;

		const double yaw = origin[2];
		tf2::Quaternion q;
		q.setRPY(0.0, 0.0, yaw);
		grid.info.origin.orientation = tf2::toMsg(q);

		grid.data.resize(static_cast<size_t>(map_width) * map_height);

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
					out = -1; // unknown in common maps
				else if (p <= occ_thr)
					out = 100;
				else if (p >= free_thr)
					out = 0;
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
				grid.data[static_cast<size_t>(x) + static_cast<size_t>(y) * map_width] = out;
			}
		}
		return grid;
	}

	nav_msgs::msg::Path createCenterPathFromCsv(const std::string &csv_file_path)
	{
		nav_msgs::msg::Path path;
		path.header.frame_id = "map";
		path.header.stamp = get_clock()->now();

		std::ifstream file(csv_file_path);
		if (!file.is_open())
		{
			RCLCPP_ERROR(get_logger(), "Failed to open path file: %s", csv_file_path.c_str());
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
					while (ss >> item)
						tokens.push_back(item);
				}
			}

			if (tokens.size() < 2)
			{
				RCLCPP_WARN(get_logger(), "Invalid line (need at least 2 numbers) at %zu: %s", line_no, line.c_str());
				continue;
			}

			try
			{
				double x = std::stod(tokens[0]);
				double y = std::stod(tokens[1]);

				geometry_msgs::msg::PoseStamped pose;
				pose.header = path.header;
				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;
				tf2::Quaternion q;
				q.setRPY(0.0, 0.0, 0.0);
				pose.pose.orientation = tf2::toMsg(q);
				path.poses.push_back(pose);
			}
			catch (const std::exception &)
			{
				RCLCPP_WARN(get_logger(), "Failed to parse numbers at line %zu: %s", line_no, line.c_str());
			}
		}

		RCLCPP_INFO(get_logger(), "Loaded %zu poses from CSV.", path.poses.size());
		return path;
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RacecarSimulator>());
	rclcpp::shutdown();
	return 0;
}
