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
#include "ackermann_msgs/msg/ackermann_drive.hpp"

using namespace std::chrono_literals;

class RacecarSimulator : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr simulator_timer;
	rclcpp::TimerBase::SharedPtr pub_timer;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive0_sub_;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive1_sub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state0_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state1_pub_;

	struct CarState
	{
		double x, y, yaw, slip_angle;
		double v, vx, vy, omega;
		double throttle, steer;
	};
	CarState car_state0, car_state1;

	struct CarParams
	{
		double mass, l_r, l_f, I_z;
		double B_f, C_f, D_f, B_r, C_r, D_r;
		double steer_max, steer_change_max, speed_max, accel_max, decel_max;
	};
	CarParams car0_params, car1_params;

	std::string drive_topic0, state_topic0, drive_topic1, state_topic1;
	double simulator_frequency, pub_frequency, friction_coefficient;

	std::vector<double> desired_speed_, desired_accel_, desired_steer_ang_;

public:
	RacecarSimulator()
		: Node("racecar_simulator")
	{

		// general parameters
		this->declare_parameter("simulator_frequency", 1000.0);
		this->get_parameter("simulator_frequency", simulator_frequency);
		this->declare_parameter("pub_frequency", 40.0);
		this->get_parameter("pub_frequency", pub_frequency);
		this->declare_parameter("friction_coefficient", 0.8);
		this->get_parameter("friction_coefficient", friction_coefficient);

		// car0 parameters
		this->declare_parameter("drive_topic0", "ackermann_cmd0");
		this->get_parameter("drive_topic0", drive_topic0);
		this->declare_parameter("state_topic0", "state0");
		this->get_parameter("state_topic0", state_topic0);
		this->declare_parameter("mass0", 3.5);
		this->get_parameter("mass0", car0_params.mass);
		this->declare_parameter("l_r0", 0.17145);
		this->get_parameter("l_r0", car0_params.l_r);
		this->declare_parameter("l_f0", 0.17145);
		this->get_parameter("l_f0", car0_params.l_f);
		this->declare_parameter("I_z0", 0.04712);
		this->get_parameter("I_z0", car0_params.I_z);
		this->declare_parameter("B_f0", 1.5);
		this->get_parameter("B_f0", car0_params.B_f);
		this->declare_parameter("C_f0", 1.5);
		this->get_parameter("C_f0", car0_params.C_f);
		this->declare_parameter("D_f0", 30.0);
		this->get_parameter("D_f0", car0_params.D_f);
		this->declare_parameter("B_r0", 1.5);
		this->get_parameter("B_r0", car0_params.B_r);
		this->declare_parameter("C_r0", 1.5);
		this->get_parameter("C_r0", car0_params.C_r);
		this->declare_parameter("D_r0", 30.0);
		this->get_parameter("D_r0", car0_params.D_r);
		this->declare_parameter("steer_max0", 0.4);
		this->get_parameter("steer_max0", car0_params.steer_max);
		this->declare_parameter("steer_change_max0", 0.041);
		this->get_parameter("steer_change_max0", car0_params.steer_change_max);
		this->declare_parameter("speed_max0", 10.0);
		this->get_parameter("speed_max0", car0_params.speed_max);
		this->declare_parameter("accel_max0", 4.0);
		this->get_parameter("accel_max0", car0_params.accel_max);
		this->declare_parameter("decel_max0", 4.0);
		this->get_parameter("decel_max0", car0_params.decel_max);

		// car1 parameters
		this->declare_parameter("drive_topic1", "ackermann_cmd1");
		this->get_parameter("drive_topic1", drive_topic1);
		this->declare_parameter("state_topic1", "state1");
		this->get_parameter("state_topic1", state_topic1);
		this->declare_parameter("mass1", 3.5);
		this->get_parameter("mass1", car1_params.mass);
		this->declare_parameter("l_r1", 0.17145);
		this->get_parameter("l_r1", car1_params.l_r);
		this->declare_parameter("l_f1", 0.17145);
		this->get_parameter("l_f1", car1_params.l_f);
		this->declare_parameter("I_z1", 0.04712);
		this->get_parameter("I_z1", car1_params.I_z);
		this->declare_parameter("B_f1", 1.5);
		this->get_parameter("B_f1", car1_params.B_f);
		this->declare_parameter("C_f1", 1.5);
		this->get_parameter("C_f1", car1_params.C_f);
		this->declare_parameter("D_f1", 30.0);
		this->get_parameter("D_f1", car1_params.D_f);
		this->declare_parameter("B_r1", 1.5);
		this->get_parameter("B_r1", car1_params.B_r);
		this->declare_parameter("C_r1", 1.5);
		this->get_parameter("C_r1", car1_params.C_r);
		this->declare_parameter("D_r1", 30.0);
		this->get_parameter("D_r1", car1_params.D_r);
		this->declare_parameter("steer_max1", 0.4);
		this->get_parameter("steer_max1", car1_params.steer_max);
		this->declare_parameter("steer_change_max1", 0.041);
		this->get_parameter("steer_change_max1", car1_params.steer_change_max);
		this->declare_parameter("speed_max1", 10.0);
		this->get_parameter("speed_max1", car1_params.speed_max);
		this->declare_parameter("accel_max1", 4.0);
		this->get_parameter("accel_max1", car1_params.accel_max);
		this->declare_parameter("decel_max1", 4.0);
		this->get_parameter("decel_max1", car1_params.decel_max);

		// Convert frequencies to durations
		auto simulator_period = std::chrono::duration<double>(1.0 / simulator_frequency);
		auto pub_period = std::chrono::duration<double>(1.0 / pub_frequency);

		// Create publishers and subscribers
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		simulator_timer = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(simulator_period),
			std::bind(&RacecarSimulator::simulator_loop, this));
		pub_timer = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(pub_period),
			std::bind(&RacecarSimulator::pub_loop, this));
		init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg)
			{ this->car0_rviz_callback(msg); });
		goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"goal_pose", 1, [this](const geometry_msgs::msg::PoseStamped::SharedPtr &msg)
			{ this->car1_rviz_callback(msg); });
		drive0_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
			drive_topic0, 1, [this](const ackermann_msgs::msg::AckermannDrive::SharedPtr &msg)
			{ this->drive0_callback(msg); });
		drive1_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
			drive_topic1, 1, [this](const ackermann_msgs::msg::AckermannDrive::SharedPtr &msg)
			{ this->drive1_callback(msg); });
		state0_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(state_topic0, 10);
		state1_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(state_topic1, 10);

		// Initialize car states
		init_cars();
	}

	void init_cars()
	{
		car_state0.x = 0.0;
		car_state0.y = 0.0;
		car_state0.yaw = 0.0;
		car_state0.slip_angle = 0.0;
		car_state0.v = 0.0;
		car_state0.vx = 0.0;
		car_state0.vy = 0.0;
		car_state0.omega = 0.0;
		car_state0.throttle = 0.0;
		car_state0.steer = 0.0;

		car_state1.x = 0.0;
		car_state1.y = 0.0;
		car_state1.yaw = 0.0;
		car_state1.slip_angle = 0.0;
		car_state1.v = 0.0;
		car_state1.vx = 0.0;
		car_state1.vy = 0.0;
		car_state1.omega = 0.0;
		car_state1.throttle = 0.0;
		car_state1.steer = 0.0;

	}

	void simulator_loop()
	{
		update_car0();
		update_car1();
	}

	void pub_loop()
	{
		state0_publisher();
		state1_publisher();
	}

	void publish_transform(const std::string &frame_id, const std::string &child_frame_id,
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

		// Send the transformation
		tf_broadcaster_->sendTransform(t);
	}

	void car0_rviz_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg)
	{
		RCLCPP_INFO(this->get_logger(), "Received initial pose for car0");

		// // Convert quaternion to Euler angles to extract yaw
		tf2::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);

		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		car_state0.x = msg->pose.pose.position.x;
		car_state0.y = msg->pose.pose.position.y;
		car_state0.yaw = yaw;
		car_state0.v = 0.0;
		car_state0.vx = 0.0;
		car_state0.vy = 0.0;
		car_state0.omega = 0.0;
		car_state0.throttle = 0.0;
		car_state0.steer = 0.0;


		publish_transform("map", "base_link0", car_state0.x,car_state0.y, car_state0.yaw);
	}

	void car1_rviz_callback(const geometry_msgs::msg::PoseStamped::SharedPtr &msg)
	{
		RCLCPP_INFO(this->get_logger(), "Received initial pose for car1");

		// Convert quaternion to Euler angles to extract yaw
		tf2::Quaternion q(
			msg->pose.orientation.x,
			msg->pose.orientation.y,
			msg->pose.orientation.z,
			msg->pose.orientation.w);

		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		car_state1.x = msg->pose.position.x;
		car_state1.y = msg->pose.position.y;
		car_state1.yaw = yaw;
		car_state1.v = 0.0;
		car_state1.vx = 0.0;
		car_state1.vy = 0.0;
		car_state1.omega = 0.0;
		car_state1.throttle = 0.0;
		car_state1.steer = 0.0;

		publish_transform("map", "base_link1", car_state1.x, car_state1.y, car_state1.yaw);
	}

	void drive0_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr &msg)
	{
		car_state0.throttle = msg->acceleration;
		car_state0.steer = msg->steering_angle;
	}

	void drive1_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr &msg)
	{
		car_state1.throttle = msg->acceleration;
		car_state1.steer = msg->steering_angle;
	}

	void state0_publisher()
	{
		// To Do: Change msg to costum message
	}

	void state1_publisher()
	{
		// To Do: Change msg to costum message
	}

	void update_car0()
	{
		// To Do: Implement the update function for car0
		publish_transform("front_left_hinge0", "front_left_wheel0", 0.0, 0.0, car_state0.steer);
		publish_transform("front_right_hinge0", "front_right_wheel0", 0.0, 0.0, car_state0.steer);
	}
	
	void update_car1()
	{
		// To Do: Implement the update function for car1
		publish_transform("front_left_hinge1", "front_left_wheel1", 0.0, 0.0, car_state1.steer);
		publish_transform("front_right_hinge1", "front_right_wheel1", 0.0, 0.0, car_state1.steer);
	}


};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RacecarSimulator>());
	rclcpp::shutdown();
	return 0;
}
