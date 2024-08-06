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

using namespace std::chrono_literals;

class RacecarSimulator : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr simulator_timer;
    rclcpp::TimerBase::SharedPtr pub_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

public:
    RacecarSimulator()
        : Node("racecar_simulator")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        simulator_timer = this->create_wall_timer(
            0.01ms, std::bind(&RacecarSimulator::simulator_loop, this));
        pub_timer = this->create_wall_timer(
            10ms, std::bind(&RacecarSimulator::pub_loop, this));

        // Corrected: Using lambda functions to capture the 'this' pointer and call member functions
        init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                this->car0_pose_callback(msg);
            });

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->car1_pose_callback(msg);
            });
		publish_transform("map", "base_link0", 1.0, -5.0, 0.0);
        publish_transform("front_left_hinge0", "front_left_wheel0", 0.0, 0.0, 0.4);
        publish_transform("front_right_hinge0", "front_right_wheel0", 0.0, 0.0, 0.3);

        publish_transform("map", "base_link1", -2.0, -5.0, 0.0);
        publish_transform("front_left_hinge1", "front_left_wheel1", 0.0, 0.0, 0.4);
        publish_transform("front_right_hinge1", "front_right_wheel1", 0.0, 0.0, 0.3);
    }

    void simulator_loop()
    {
        // publish_transform("map", "base_link0", 1.0, -5.0, 0.0);
        publish_transform("front_left_hinge0", "front_left_wheel0", 0.0, 0.0, 0.4);
        publish_transform("front_right_hinge0", "front_right_wheel0", 0.0, 0.0, 0.3);

        // publish_transform("map", "base_link1", -2.0, -5.0, 0.0);
        publish_transform("front_left_hinge1", "front_left_wheel1", 0.0, 0.0, 0.4);
        publish_transform("front_right_hinge1", "front_right_wheel1", 0.0, 0.0, 0.3);
    }

    void pub_loop()
    {
        // Logic for publishing other data (if needed) can be added here
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

    void car0_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received initial pose for car0");

        // Convert quaternion to Euler angles to extract yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        publish_transform("map", "base_link0",
                          msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          yaw); // Use extracted yaw
    }

    void car1_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

        publish_transform("map", "base_link1",
                          msg->pose.position.x,
                          msg->pose.position.y,
                          yaw); // Use extracted yaw
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacecarSimulator>());
    rclcpp::shutdown();
    return 0;
}
