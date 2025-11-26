#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <chrono>
#include <string>
#include <algorithm>

class CenterLookaheadSineSweepNode : public rclcpp::Node {
public:
  CenterLookaheadSineSweepNode() : Node("center_lookahead_sine_sweep_node") {
    using std::chrono::milliseconds;

    // === 차량/입력 한계 ===
    wheelbase_ = declare_parameter<double>("wheelbase", 0.46);   // [m] (지금은 거의 안 씀)
    a_min_     = declare_parameter<double>("accel_min", -10.0);  // [m/s^2] 최댓 감속 (음수)
    a_max_     = declare_parameter<double>("accel_max",  10.0);  // [m/s^2] 최댓 가속
    steer_max_ = declare_parameter<double>("steer_max",  0.4189);// [rad] ≈ 24 deg

    // 원 중심을 바라보게 하는 조향 gain
    k_steer_center_ = declare_parameter<double>("k_steer_center", 0.01);

    // 최소 속도 유지 파라미터
    v_min_target_ = declare_parameter<double>("v_min_target", 1.0);   // [m/s] 유지하고 싶은 최소 속도
    k_v_min_      = declare_parameter<double>("k_v_min",     2.0);    // (v_min - v)용 P 게인

    // === 안전 원 파라미터 ===
    R_safe_   = declare_parameter<double>("safe_radius", 500.0);    // [m]
    margin_   = declare_parameter<double>("safe_margin",  50.0);    // [m] 경계 안쪽 guard band
    center_x_ = declare_parameter<double>("circle_center_x", 0.0);
    center_y_ = declare_parameter<double>("circle_center_y", 0.0);

    // === Sine Sweep 파라미터 (종방향만 사용) ===
    A_long_max_   = declare_parameter<double>("long_sweep_amplitude", 5.0);   // [m/s^2]
    f_long_start_ = declare_parameter<double>("long_f_start", 0.001);           // [Hz]
    f_long_end_   = declare_parameter<double>("long_f_end",   0.5);           // [Hz]
    sweep_duration_ = declare_parameter<double>("sweep_duration", 60.0);      // [s]

    odom_topic_  = declare_parameter<std::string>("odom_topic",  "odom0");
    drive_topic_ = declare_parameter<std::string>("drive_topic", "ackermann_cmd0");

    if (a_min_ >= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "accel_min (a_min_) is not negative. For safety, set to -5.0 m/s^2");
      a_min_ = -5.0;
    }

    start_time_ = now();

    // QoS
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, pub_qos);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sub_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg){
          odom_ = *msg;
          has_odom_ = true;
        });

    timer_ = create_wall_timer(milliseconds(10),
                               std::bind(&CenterLookaheadSineSweepNode::onTimer, this));
  }

private:
  void onTimer() {
    if (!has_odom_) return;

    // === 현재 pose & yaw ===
    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;

    double roll, pitch, yaw;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);

    const double x = p.x;
    const double y = p.y;
    const double v = odom_.twist.twist.linear.x;

    // === 원 좌표계 ===
    const double dx_c = x - center_x_;
    const double dy_c = y - center_y_;
    const double r    = std::hypot(dx_c, dy_c);
    const double d    = R_safe_ - r;        // 경계까지 거리 (양수면 안쪽, 0이면 경계)

    // === 바깥 방향 속도 성분 v_r ===
    double v_r = 0.0;  // >0 이면 원 밖 방향으로 진행 중
    if (r > 1e-3) {
      const double ex = dx_c / r;      // radial unit outward x
      const double ey = dy_c / r;      // radial unit outward y
      const double vx = v * std::cos(yaw);
      const double vy = v * std::sin(yaw);
      v_r = vx * ex + vy * ey;
    }

    // === 1. 조향: 항상 원 중심을 lookahead point 로 보는 형태 ===
    double angle_to_center = std::atan2(center_y_ - y, center_x_ - x);
    double yaw_err = normalizeAngle(angle_to_center - yaw);
    double steer_cmd = k_steer_center_ * yaw_err;
    steer_cmd = std::max(-steer_max_, std::min(steer_max_, steer_cmd));

    // === 2. 종방향 Sine Sweep + 속도 바닥 제어 ===
    const double t = (now() - start_time_).seconds();

    auto chirp = [](double f_start, double f_end, double T, double t_now){
      if (T <= 0.0) return f_start;
      if (t_now >= T) return f_end;
      double s = t_now / T;  // 0~1
      return f_start + (f_end - f_start) * s;
    };

    const double f_long = chirp(f_long_start_, f_long_end_, sweep_duration_, t);
    constexpr double PI = 3.14159265358979323846;

    // 순수 sine sweep
    double a_sweep = A_long_max_ * std::sin(2.0 * PI * f_long * t);

    // 최소 속도 유지: 원 내부 깊은 영역에서만 적용
    double a_floor = 0.0;
    if (d > margin_) {
      if (v < v_min_target_) {
        a_floor = k_v_min_ * (v_min_target_ - v);
      }
    }

    double a_cmd = a_sweep + a_floor;

    // 기본 한계 clip
    a_cmd = std::max(a_min_, std::min(a_max_, a_cmd));

    // === 3. 안전 제어 (원 밖으로 안 나가게 override) ===
    const double a_brake = std::abs(a_min_); // 사용할 수 있는 최대 감속 [>0]

    // (A) 이미 원 밖인 경우: 풀 브레이크 + 중심 조향 (steer는 이미 center를 향함)
    if (d <= 0.0) {
      a_cmd = a_min_; // 최대 감속

      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Outside safe circle! r=%.3f > R_safe=%.3f. Emergency brake.",
          r, R_safe_);
    }
    // (B) guard band 안쪽 (R_safe - margin ~ R_safe):
    // 바깥 방향 속도 v_r와 d 로 stopping distance 체크
    else if (d <= margin_) {
      if (v_r > 0.0) {
        double s_stop = (v_r * v_r) / (2.0 * a_brake); // 바깥 방향 제동거리

        if (s_stop >= d) {
          // 지금 속도로 가면 경계를 넘으니, sine sweep 무시하고 풀 브레이크
          a_cmd = a_min_;
        } else {
          // 아직 여유는 있지만, 경계에 가까워질수록 진폭 줄이기
          double scale = d / margin_;  // 1 -> 0
          a_cmd *= scale;
        }
      } else {
        // v_r <= 0: 안쪽/접선 방향 → r 증가 위험 적음, 그래도 살짝 줄이기
        double scale = d / margin_;
        a_cmd *= scale;
      }
    }
    // (C) 내부 영역 (r <= R_safe - margin): a_cmd = sweep + floor 그대로, safety override 없음

    // 최종 saturation
    if (a_cmd > a_max_) a_cmd = a_max_;
    if (a_cmd < a_min_) a_cmd = a_min_;
    if (steer_cmd >  steer_max_) steer_cmd =  steer_max_;
    if (steer_cmd < -steer_max_) steer_cmd = -steer_max_;

    // === Publish ===
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer_cmd;
    cmd.drive.acceleration   = a_cmd;
    // cmd.drive.speed 는 여기서 사용 X

    drive_pub_->publish(cmd);
  }

  static double normalizeAngle(double a) {
    constexpr double PI = 3.14159265358979323846;
    while (a >  PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
  }

  // Members
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry odom_;
  bool has_odom_{false};

  // 파라미터들
  double wheelbase_;
  double a_min_, a_max_;
  double steer_max_;
  double k_steer_center_;
  double R_safe_, margin_;
  double center_x_, center_y_;

  // 최소 속도 유지
  double v_min_target_;
  double k_v_min_;

  // 종방향 sine sweep
  double A_long_max_, f_long_start_, f_long_end_;
  double sweep_duration_;

  std::string odom_topic_, drive_topic_;
  rclcpp::Time start_time_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterLookaheadSineSweepNode>());
  rclcpp::shutdown();
  return 0;
}
