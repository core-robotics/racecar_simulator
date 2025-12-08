#include <cmath>
#include <deque>
#include <string>
#include <vector>
#include <algorithm>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Eigen for small least-squares polyfit
#include <Eigen/Dense>

namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double angle_wrap(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// Push with maxlen behavior
void push_limited(std::deque<double> & buf, double v, std::size_t max_len)
{
  buf.push_back(v);
  while (buf.size() > max_len) {
    buf.pop_front();
  }
}

// median helper
double median_of(std::vector<double> v)
{
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  if (n % 2 == 1) {
    return v[n / 2];
  } else {
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
  }
}

// Hampel filter (last sample correction)
// if last is outlier -> replace with median
double hampel_last(const std::deque<double> & values, int window, double n_sigma)
{
  const int n = static_cast<int>(values.size());
  if (n <= 0) return 0.0;
  if (window < 3) return values.back();

  const int w = std::min(window, n);
  const int start = n - w;

  std::vector<double> win;
  win.reserve(static_cast<std::size_t>(w));
  for (int i = start; i < n; ++i) {
    win.push_back(values[static_cast<std::size_t>(i)]);
  }

  const double med = median_of(win);

  std::vector<double> abs_dev;
  abs_dev.reserve(win.size());
  for (double x : win) {
    abs_dev.push_back(std::abs(x - med));
  }

  const double mad = median_of(abs_dev);
  const double sigma = 1.4826 * mad;  // consistent with normal dist

  if (sigma < 1e-12) {
    return values.back();
  }

  const double x_last = values.back();
  if (std::abs(x_last - med) > n_sigma * sigma) {
    return med;
  }
  return x_last;
}

// Online Savitzky–Golay-like smoothing:
// Fit polynomial of given order to last N samples and
// return estimated value at last sample index.
double online_sg_last(const std::deque<double> & values, int order)
{
  const int n = static_cast<int>(values.size());
  if (n <= 0) {
    return 0.0;
  }
  if (n <= order) {
    return values.back();
  }

  Eigen::VectorXd y(n);
  for (int i = 0; i < n; ++i) {
    y(i) = values[static_cast<std::size_t>(i)];
  }

  Eigen::MatrixXd A(n, order + 1);
  for (int i = 0; i < n; ++i) {
    double xp = 1.0;
    for (int j = 0; j <= order; ++j) {
      A(i, j) = xp;
      xp *= static_cast<double>(i);
    }
  }

  Eigen::VectorXd coeff = A.colPivHouseholderQr().solve(y);

  const double x_last = static_cast<double>(n - 1);
  double y_hat = 0.0;
  double xp = 1.0;
  for (int j = 0; j <= order; ++j) {
    y_hat += coeff(j) * xp;
    xp *= x_last;
  }

  return y_hat;
}

}  // namespace


class MocapPoseToOdom : public rclcpp::Node
{
public:
  MocapPoseToOdom()
  : Node("mocap_pose_to_odom")
  {
    // Topics/frames
    this->declare_parameter<std::string>("pose_topic", "/mocap_pose");
    this->declare_parameter<std::string>("odom_topic", "/mocap_odom");
    this->declare_parameter<std::string>("frame_id", "odom");
    this->declare_parameter<std::string>("child_frame_id", "base_link");

    // Physical velocity limits
    this->declare_parameter<double>("max_vx", 10.0);
    this->declare_parameter<double>("max_vy", 3.0);
    this->declare_parameter<double>("max_wz", 6.0);

    // Δ clamp margin
    this->declare_parameter<double>("step_scale", 1.5);
    this->declare_parameter<double>("yaw_step_scale", 1.5);

    // SG window (for twist)
    this->declare_parameter<int>("sg_window", 7);

    // Hampel params (for twist)
    this->declare_parameter<int>("hampel_window", 7);
    this->declare_parameter<double>("hampel_nsigma", 3.0);

    pose_topic_ = this->get_parameter("pose_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();

    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();

    max_vx_ = this->get_parameter("max_vx").as_double();
    max_vy_ = this->get_parameter("max_vy").as_double();
    max_wz_ = this->get_parameter("max_wz").as_double();

    step_scale_ = this->get_parameter("step_scale").as_double();
    yaw_step_scale_ = this->get_parameter("yaw_step_scale").as_double();

    sg_window_ = this->get_parameter("sg_window").as_int();
    if (sg_window_ < 5) sg_window_ = 5;
    if (sg_window_ % 2 == 0) sg_window_ += 1;
    sg_order_ = 3;

    hampel_window_ = this->get_parameter("hampel_window").as_int();
    if (hampel_window_ < 3) hampel_window_ = 3;
    if (hampel_window_ % 2 == 0) hampel_window_ += 1;
    hampel_nsigma_ = this->get_parameter("hampel_nsigma").as_double();

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MocapPoseToOdom::pose_cb, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Subscribing: %s (QoS: sensor_data/BEST_EFFORT)", pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing:  %s", odom_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Δ-clamp + Twist Hampel->SG3 enabled: "
      "max_vx=%.3f, max_vy=%.3f, max_wz=%.3f, step_scale=%.3f, yaw_step_scale=%.3f, "
      "twist_sg_window=%d, hampel_window=%d, hampel_nsigma=%.2f",
      max_vx_, max_vy_, max_wz_, step_scale_, yaw_step_scale_,
      sg_window_, hampel_window_, hampel_nsigma_
    );
  }

private:
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    rclcpp::Time curr_time = rclcpp::Time(msg->header.stamp);

    if (curr_time.seconds() == 0.0) {
      curr_time = this->now();
    }

    const double x_raw = msg->pose.position.x;
    const double y_raw = msg->pose.position.y;
    const double yaw_raw = yaw_from_quaternion(msg->pose.orientation);

    // First frame init
    if (!c_x_.has_value()) {
      prev_time_ = curr_time;

      c_x_ = x_raw;
      c_y_ = y_raw;
      c_yaw_unwrapped_ = yaw_raw;

      // init twist buffers
      push_limited(buf_vx_raw_, 0.0, static_cast<std::size_t>(hampel_window_));
      push_limited(buf_vy_raw_, 0.0, static_cast<std::size_t>(hampel_window_));
      push_limited(buf_wz_raw_, 0.0, static_cast<std::size_t>(hampel_window_));

      push_limited(buf_vx_sg_, 0.0, static_cast<std::size_t>(sg_window_));
      push_limited(buf_vy_sg_, 0.0, static_cast<std::size_t>(sg_window_));
      push_limited(buf_wz_sg_, 0.0, static_cast<std::size_t>(sg_window_));

      const double yaw_pub = angle_wrap(*c_yaw_unwrapped_);
      auto odom = build_odom(*msg, *c_x_, *c_y_, yaw_pub, 0.0, 0.0, 0.0);
      pub_->publish(odom);
      return;
    }

    // dt
    const double dt = (curr_time - *prev_time_).seconds();
    if (dt <= 0.0) {
      const double yaw_pub = angle_wrap(*c_yaw_unwrapped_);
      auto odom = build_odom(*msg, *c_x_, *c_y_, yaw_pub, 0.0, 0.0, 0.0);
      pub_->publish(odom);
      return;
    }

    // ---- 1) Δ clamp only ----
    const double max_speed = std::hypot(max_vx_, max_vy_);
    const double max_step_dist = max_speed * dt * step_scale_;
    const double max_step_yaw  = std::abs(max_wz_) * dt * yaw_step_scale_;

    const double dx_raw = x_raw - *c_x_;
    const double dy_raw = y_raw - *c_y_;
    const double dist = std::hypot(dx_raw, dy_raw);

    double x_c = x_raw;
    double y_c = y_raw;
    if (dist > 1e-9 && dist > max_step_dist) {
      const double scale = max_step_dist / dist;
      x_c = *c_x_ + dx_raw * scale;
      y_c = *c_y_ + dy_raw * scale;
    }

    // yaw clamp step in wrapped space, then update unwrapped
    const double c_yaw_wrapped_old = angle_wrap(*c_yaw_unwrapped_);
    double dyaw_wrapped = angle_wrap(yaw_raw - c_yaw_wrapped_old);
    if (std::abs(dyaw_wrapped) > max_step_yaw) {
      dyaw_wrapped = std::copysign(max_step_yaw, dyaw_wrapped);
    }
    const double yaw_unwrapped_c = *c_yaw_unwrapped_ + dyaw_wrapped;

    // ---- 2) Compute raw twist from CLAMPED pose delta ----
    const double dx_c = x_c - *c_x_;
    const double dy_c = y_c - *c_y_;

    const double yaw_wrapped_new = angle_wrap(yaw_unwrapped_c);
    const double dyaw_c = angle_wrap(yaw_wrapped_new - c_yaw_wrapped_old);

    double vx_raw = dx_c / dt;
    double vy_raw = dy_c / dt;
    double wz_raw = dyaw_c / dt;

    // ---- 3) Update clamped reference AFTER delta used ----
    c_x_ = x_c;
    c_y_ = y_c;
    c_yaw_unwrapped_ = yaw_unwrapped_c;

    // ---- 4) Hampel (once) on raw twist ----
    push_limited(buf_vx_raw_, vx_raw, static_cast<std::size_t>(hampel_window_));
    push_limited(buf_vy_raw_, vy_raw, static_cast<std::size_t>(hampel_window_));
    push_limited(buf_wz_raw_, wz_raw, static_cast<std::size_t>(hampel_window_));

    const double vx_h = hampel_last(buf_vx_raw_, hampel_window_, hampel_nsigma_);
    const double vy_h = hampel_last(buf_vy_raw_, hampel_window_, hampel_nsigma_);
    const double wz_h = hampel_last(buf_wz_raw_, hampel_window_, hampel_nsigma_);

    // ---- 5) SavGol (online polyfit) on Hampel output ----
    push_limited(buf_vx_sg_, vx_h, static_cast<std::size_t>(sg_window_));
    push_limited(buf_vy_sg_, vy_h, static_cast<std::size_t>(sg_window_));
    push_limited(buf_wz_sg_, wz_h, static_cast<std::size_t>(sg_window_));

    double vx = online_sg_last(buf_vx_sg_, sg_order_);
    double vy = online_sg_last(buf_vy_sg_, sg_order_);
    double wz = online_sg_last(buf_wz_sg_, sg_order_);

    // ---- 6) physical clamps (final) ----
    vx = std::max(-max_vx_, std::min(max_vx_, vx));
    vy = std::max(-max_vy_, std::min(max_vy_, vy));
    wz = std::max(-max_wz_, std::min(max_wz_, wz));

    prev_time_ = curr_time;

    // Publish pose as CLAMPED (no pose SG)
    const double yaw_pub = angle_wrap(*c_yaw_unwrapped_);
    auto odom = build_odom(*msg, *c_x_, *c_y_, yaw_pub, vx, vy, wz);
    pub_->publish(odom);
  }

  nav_msgs::msg::Odometry build_odom(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    double x, double y, double yaw,
    double vx, double vy, double wz)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = pose_msg.header.stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    // pose
    odom.pose.pose = pose_msg.pose;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;

    // orientation:
    // Python과 동일하게 원본 quaternion 유지
    odom.pose.pose.orientation = pose_msg.pose.orientation;

    // twist
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = wz;

    (void)yaw; // kept for future extension if you reconstruct quaternion from yaw

    return odom;
  }

private:
  // params/topics
  std::string pose_topic_;
  std::string odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  double max_vx_{10.0};
  double max_vy_{3.0};
  double max_wz_{6.0};

  double step_scale_{1.5};
  double yaw_step_scale_{1.5};

  int sg_window_{7};
  int sg_order_{3};

  int hampel_window_{7};
  double hampel_nsigma_{3.0};

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  // time/state
  std::optional<rclcpp::Time> prev_time_;

  // Last clamped raw
  std::optional<double> c_x_;
  std::optional<double> c_y_;
  std::optional<double> c_yaw_unwrapped_;

  // Twist buffers: raw for Hampel
  std::deque<double> buf_vx_raw_;
  std::deque<double> buf_vy_raw_;
  std::deque<double> buf_wz_raw_;

  // Twist buffers: Hampel output for SG
  std::deque<double> buf_vx_sg_;
  std::deque<double> buf_vy_sg_;
  std::deque<double> buf_wz_sg_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MocapPoseToOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
