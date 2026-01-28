#include <cmath>
#include <chrono>
#include <algorithm>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    wheelbase_      = declare_parameter("wheelbase", 0.33);

    lookahead_min_  = declare_parameter("lookahead_min", 0.5);
    lookahead_max_  = declare_parameter("lookahead_max", 3.0);
    lookahead_gain_ = declare_parameter("lookahead_gain", 0.4);

    v_max_          = declare_parameter("v_max", 6.0);
    v_min_          = declare_parameter("v_min", 1.0);
    lat_accel_max_  = declare_parameter("lat_accel_max", 2.5);

    steer_rate_max_ = declare_parameter("steer_rate_max", 2.0);
    pad_points_     = declare_parameter("pad_points", 20); 

    path_topic_     = declare_parameter("path_topic",  "center_path");
    pose_topic_     = declare_parameter("pose_topic",  "pose0");
    odom_topic_     = declare_parameter("odom_topic",  "odom0");
    drive_topic_    = declare_parameter("drive_topic", "ackermann_cmd0");

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 1);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
        if (path_.poses.size() < 2) return;

        const size_t k = std::min<size_t>(std::max(0, pad_points_), path_.poses.size());
        for (size_t i = 0; i < k; ++i) path_.poses.push_back(path_.poses[i]);
      });

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(10).best_effort(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_ = *msg; has_pose_ = true; });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10).best_effort(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = *msg; has_odom_ = true; });

    last_time_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { controlLoop(); });
  }

private:
  void controlLoop() {
    if (!has_pose_ || !has_odom_ || path_.poses.size() < 2) return;

    const auto t = now();
    const double dt = std::max(1e-3, (t - last_time_).seconds());
    last_time_ = t;

    const auto &p = pose_.pose.position;
    const auto &q = pose_.pose.orientation;
    const double yaw = yawFromQuat(q.x, q.y, q.z, q.w);
    const double v   = odom_.twist.twist.linear.x;

    const double lookahead = std::clamp(lookahead_min_ + lookahead_gain_ * std::abs(v),
                                        lookahead_min_, lookahead_max_);

    double tx = 0.0, ty = 0.0;
    if (!lookaheadPoint(path_, p.x, p.y, lookahead, tx, ty)) return;

    const double dx = tx - p.x, dy = ty - p.y;
    const double x_local =  std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double y_local = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (x_local <= 0.0) return;

    const double Ld = std::hypot(x_local, y_local);
    const double curvature = 2.0 * y_local / (Ld * Ld);

    double steer = std::atan(wheelbase_ * curvature);
    steer = rateLimit(steer, last_steer_, steer_rate_max_, dt);
    last_steer_ = steer;

    const double kappa = std::max(1e-4, std::abs(curvature));
    const double v_ref = std::clamp(std::sqrt(lat_accel_max_ / kappa), v_min_, v_max_);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = t;
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer;
    cmd.drive.speed = v_ref;

    std::cout << "Lookahead: " << lookahead << "\n"
              << "Steer: " << steer << "\n"
              << "V_ref: " << v_ref << "\n"
              << "V_curr: " << v << "\n"
              <<"\n";

    drive_pub_->publish(cmd);
  }

  static double yawFromQuat(double x, double y, double z, double w) {
    double r, p, yaw;
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    return yaw;
  }

  static double rateLimit(double desired, double prev, double rate_max, double dt) {
    const double max_delta = std::max(0.0, rate_max) * dt;
    return prev + std::clamp(desired - prev, -max_delta, max_delta);
  }

  static bool lookaheadPoint(const nav_msgs::msg::Path &path,
                             double x, double y, double lookahead,
                             double &out_x, double &out_y) {
    const size_t N = path.poses.size();
    if (N < 2) return false;

    auto P = [](const geometry_msgs::msg::PoseStamped &ps) { return ps.pose.position; };

    size_t best_i = 0;
    double best_t = 0.0, best_d2 = 1e18;

    for (size_t i = 0; i + 1 < N; ++i) {
      const auto a = P(path.poses[i]);
      const auto b = P(path.poses[i + 1]);

      const double vx = b.x - a.x, vy = b.y - a.y;
      const double vv = vx * vx + vy * vy;
      if (vv < 1e-12) continue;

      const double wx = x - a.x, wy = y - a.y;
      const double t = std::clamp((wx * vx + wy * vy) / vv, 0.0, 1.0);

      const double px = a.x + t * vx, py = a.y + t * vy;
      const double dx = x - px, dy = y - py;
      const double d2 = dx * dx + dy * dy;

      if (d2 < best_d2) { best_d2 = d2; best_i = i; best_t = t; }
    }

    const auto a0 = P(path.poses[best_i]);
    const auto b0 = P(path.poses[best_i + 1]);
    double cx = a0.x + best_t * (b0.x - a0.x);
    double cy = a0.y + best_t * (b0.y - a0.y);

    double remain = lookahead;
    for (size_t i = best_i; i + 1 < N; ++i) {
      const auto b = P(path.poses[i + 1]);
      const double seg = std::hypot(b.x - cx, b.y - cy);

      if (seg < 1e-12) { cx = b.x; cy = b.y; continue; }
      if (seg >= remain) {
        const double t = remain / seg;
        out_x = cx + t * (b.x - cx);
        out_y = cy + t * (b.y - cy);
        return true;
      }
      remain -= seg;
      cx = b.x; cy = b.y;
    }

    out_x = cx; out_y = cy;
    return true;
  }

private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped pose_;
  nav_msgs::msg::Odometry odom_;
  bool has_pose_{false}, has_odom_{false};

  double wheelbase_, lookahead_min_, lookahead_max_, lookahead_gain_;
  double v_max_, v_min_, lat_accel_max_, steer_rate_max_;
  int pad_points_;

  std::string path_topic_, pose_topic_, odom_topic_, drive_topic_;

  rclcpp::Time last_time_;
  double last_steer_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
