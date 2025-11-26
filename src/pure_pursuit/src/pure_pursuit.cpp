// pure_pursuit_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    // Params
    lookahead_ = declare_parameter<double>("lookahead", 1.5);     // [m]
    wheelbase_ = declare_parameter<double>("wheelbase", 0.34);    // [m]
    v_min_     = declare_parameter<double>("speed_min", 2.0);     // [m/s]
    v_max_     = declare_parameter<double>("speed_max", 5.0);     // [m/s]
    k_speed_   = declare_parameter<double>("k_speed",  3.0);      // curvature → speed
    k_accel_   = declare_parameter<double>("k_accel",  2.0);      // P gain for (v_ref - v)
    a_min_     = declare_parameter<double>("accel_min",-10.0);     // [m/s^2]
    a_max_     = declare_parameter<double>("accel_max", 10.0);     // [m/s^2]
    center_path_topic_ = declare_parameter<std::string>("center_path_topic", "center_path");
    left_path_topic_   = declare_parameter<std::string>("left_boundary", "left_boundary");
    right_path_topic_  = declare_parameter<std::string>("right_boundary","right_boundary");
    odom_topic_        = declare_parameter<std::string>("odom_topic", "odom0");
    drive_topic_       = declare_parameter<std::string>("drive_topic", "ackermann_cmd0");

    // Pubs/Subs
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, pub_qos);

    sub_center_ = create_subscription<nav_msgs::msg::Path>(
        center_path_topic_, map_qos,
        [this](nav_msgs::msg::Path::SharedPtr msg){ center_path_ = *msg; });

    sub_left_ = create_subscription<nav_msgs::msg::Path>(
        left_path_topic_, map_qos,
        [this](nav_msgs::msg::Path::SharedPtr msg){ left_path_ = *msg; });

    sub_right_ = create_subscription<nav_msgs::msg::Path>(
        right_path_topic_, map_qos,
        [this](nav_msgs::msg::Path::SharedPtr msg){ right_path_ = *msg; });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sub_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg){
          odom_ = *msg; has_odom_ = true;
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&PurePursuitNode::onTimer, this));
  }

private:
  void onTimer() {
    if (!has_odom_ || center_path_.poses.empty()) return;

    // Current pose & yaw
    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    const double x = p.x, y = p.y;

    // === 변경점: Path를 원형으로 간주하여 lookahead 타겟 인덱스 선택 ===
    int target_idx = findLookaheadIndexCyclic(center_path_, x, y, lookahead_);
    if (target_idx < 0) return;

    const auto &tp = center_path_.poses[target_idx].pose.position;

    // Target in vehicle frame (x forward, y left)
    const double dx = tp.x - x;
    const double dy = tp.y - y;
    const double xL =  std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double yL = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (xL <= 0.01) return;

    // Pure Pursuit geometry
    const double Ld = std::hypot(xL, yL);
    const double curvature = 2.0 * yL / (Ld * Ld);
    const double steer = std::atan(wheelbase_ * curvature);

    // Speed target from curvature
    double v_ref = std::max(v_min_, v_max_ - k_speed_ * std::abs(curvature));
    v_ref = std::min(v_ref, v_max_);

    // Current longitudinal speed
    const double v = odom_.twist.twist.linear.x;

    // Acceleration command (P-control), clamped
    double a_cmd = k_accel_ * (v_ref - v);
    if (a_cmd > a_max_) a_cmd = a_max_;
    if (a_cmd < a_min_) a_cmd = a_min_;

    // Publish acceleration-based command
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer;
    cmd.drive.acceleration   = a_cmd;
    // Optional: set speed field unused or as reference
    // cmd.drive.speed = 0.0;

    drive_pub_->publish(cmd);
  }


  static int findLookaheadIndexCyclic(const nav_msgs::msg::Path &path,
                                      double x, double y, double Ld) {
    const size_t N = path.poses.size();
    if (N == 0) return -1;
    if (N == 1) return 0;
    size_t closest = 0;
    double best_d2 = 1e18;
    for (size_t i = 0; i < N; ++i) {
      const auto &pt = path.poses[i].pose.position;
      double d2 = (pt.x - x) * (pt.x - x) + (pt.y - y) * (pt.y - y);
      if (d2 < best_d2) { best_d2 = d2; closest = i; }
    }

    double accum = 0.0;
    for (size_t step = 0; step < N; ++step) {
      size_t i     = (closest + step) % N;
      size_t inext = (i + 1) % N;
      const auto &a = path.poses[i].pose.position;
      const auto &b = path.poses[inext].pose.position;
      accum += std::hypot(b.x - a.x, b.y - a.y);
      if (accum >= Ld) return static_cast<int>(inext);
    }

    return static_cast<int>(closest);
  }

  // Members
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_center_, sub_left_, sub_right_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path center_path_, left_path_, right_path_;
  nav_msgs::msg::Odometry odom_;
  bool has_odom_{false};

  // Params
  double lookahead_, wheelbase_;
  double v_min_, v_max_, k_speed_;
  double k_accel_, a_min_, a_max_;
  std::string center_path_topic_, left_path_topic_, right_path_topic_, odom_topic_, drive_topic_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
