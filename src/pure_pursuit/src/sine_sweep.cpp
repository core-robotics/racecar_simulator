#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <chrono>
#include <string>
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>

class GridSweepExcitationNode : public rclcpp::Node {
public:
  GridSweepExcitationNode()
  : Node("grid_sweep_excitation_node"),
    rng_(std::random_device{}())
  {
    using std::chrono::milliseconds;

    // Vehicle limits
    wheelbase_ = declare_parameter<double>("wheelbase", 0.46);
    a_min_     = declare_parameter<double>("accel_min", -5.0);
    a_max_     = declare_parameter<double>("accel_max",  5.0);
    steer_max_ = declare_parameter<double>("steer_max",  0.4189);

    if (a_min_ >= 0.0) {
      RCLCPP_WARN(get_logger(), "accel_min must be negative, clamping to -5.0");
      a_min_ = -5.0;
    }
    if (a_max_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "accel_max must be positive, clamping to 5.0");
      a_max_ = 5.0;
    }
    if (a_min_ >= a_max_) {
      RCLCPP_WARN(get_logger(), "accel_min >= accel_max, resetting to [-5,5]");
      a_min_ = -5.0;
      a_max_ =  5.0;
    }

    // v_ref, w_ref ranges  (v: 0.5~10, w: -5~+5)
    v_min_ref_ = declare_parameter<double>("v_min_ref", 0.5);
    v_max_ref_ = declare_parameter<double>("v_max_ref", 10.0);
    w_max_ref_ = declare_parameter<double>("w_max_ref", 5.0);

    if (v_min_ref_ > v_max_ref_) {
      RCLCPP_WARN(get_logger(), "v_min_ref > v_max_ref, swapping");
      std::swap(v_min_ref_, v_max_ref_);
    }
    if (w_max_ref_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "w_max_ref must be positive, clamping to 1.0");
      w_max_ref_ = 1.0;
    }

    // Grid config
    n_v_            = declare_parameter<int>("n_v", 7);
    n_w_            = declare_parameter<int>("n_w", 7);
    dwell_time_sec_ = declare_parameter<double>("dwell_time", 10.0);
    shuffle_grid_   = declare_parameter<bool>("shuffle_grid", true);

    // v_ref sine sweep (global max amplitude/freq)
    A_v_max_   = declare_parameter<double>("v_sweep_amplitude", 5.0);  // 크게
    f_v_start_ = declare_parameter<double>("v_f_start", 0.05);
    f_v_end_   = declare_parameter<double>("v_f_end",   1.0);

    // w_ref sine sweep (global max amplitude/freq)
    A_w_max_   = declare_parameter<double>("w_sweep_amplitude", 5.0);  // 크게
    f_w_start_ = declare_parameter<double>("w_f_start", 0.05);
    f_w_end_   = declare_parameter<double>("w_f_end",   1.0);

    // Tracking gains
    k_v_ = declare_parameter<double>("k_v", 1.0);
    k_w_ = declare_parameter<double>("k_w", 0.5);

    // Amplitude sanity check
    const double v_range = v_max_ref_ - v_min_ref_;
    if (A_v_max_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "v_sweep_amplitude <= 0, setting to half of v-range");
      A_v_max_ = 0.5 * v_range;
    }
    if (A_v_max_ > v_range) {
      RCLCPP_WARN(get_logger(),
                  "v_sweep_amplitude > v-range, clamping to v-range");
      A_v_max_ = v_range;
    }
    if (A_w_max_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "w_sweep_amplitude <= 0, setting to w_max_ref");
      A_w_max_ = w_max_ref_;
    }
    if (A_w_max_ > w_max_ref_) {
      RCLCPP_WARN(get_logger(),
                  "w_sweep_amplitude > w_max_ref, clamping");
      A_w_max_ = w_max_ref_;
    }

    // Topics
    odom_topic_  = declare_parameter<std::string>("odom_topic",  "odom0");
    drive_topic_ = declare_parameter<std::string>("drive_topic", "ackermann_cmd0");

    buildGridPattern();

    last_switch_time_ = std::chrono::steady_clock::now();
    current_index_    = 0;

    // ROS interfaces
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, pub_qos);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sub_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_ = *msg;
          has_odom_ = true;
        });

    timer_ = create_wall_timer(
        milliseconds(10),
        std::bind(&GridSweepExcitationNode::onTimerTick, this));
  }

private:
  struct GridCellCommand {
    double v_bias;
    double w_bias;
    double v_amp;   // 이 셀에서 사용할 v amplitude
    double w_amp;   // 이 셀에서 사용할 w amplitude
  };

  template <typename T>
  static T clamp(T value, T lo, T hi) {
    return std::max(lo, std::min(hi, value));
  }

  static double computeSweepFrequency(double f_start, double f_end,
                                      double T, double t_now) {
    if (T <= 0.0)   return f_start;
    if (t_now >= T) return f_end;
    const double s = t_now / T;
    return f_start + (f_end - f_start) * s;
  }

  void buildGridPattern() {
    inputs_.clear();

    n_v_ = std::max(n_v_, 1);
    n_w_ = std::max(n_w_, 1);

    const double v_min = v_min_ref_;
    const double v_max = v_max_ref_;
    const double w_min = -w_max_ref_;
    const double w_max =  w_max_ref_;

    const double d_v = (n_v_ > 1) ? (v_max - v_min) / (n_v_ - 1) : 0.0;
    const double d_w = (n_w_ > 1) ? (w_max - w_min) / (n_w_ - 1) : 0.0;

    for (int i = 0; i < n_v_; ++i) {
      const double v_bias = v_min + d_v * i;
      for (int j = 0; j < n_w_; ++j) {
        const double w_bias = w_min + d_w * j;

        // 이 셀에서 v,w가 범위를 넘지 않으면서 최대한 크게 흔들리도록 amplitude 설정
        const double v_margin_low  = v_bias - v_min;
        const double v_margin_high = v_max - v_bias;
        const double v_amp_cell    = std::min({A_v_max_, v_margin_low, v_margin_high});

        const double w_margin_low  = w_bias - w_min; // = w_bias + w_max_ref_
        const double w_margin_high = w_max - w_bias;
        const double w_amp_cell    = std::min({A_w_max_, w_margin_low, w_margin_high});

        inputs_.push_back({v_bias, w_bias, v_amp_cell, w_amp_cell});
      }
    }

    if (shuffle_grid_ && inputs_.size() > 1) {
      std::shuffle(inputs_.begin(), inputs_.end(), rng_);
    }

    const std::size_t total_cells = inputs_.size();
    total_cycle_time_sec_ = dwell_time_sec_ * static_cast<double>(total_cells);

    RCLCPP_INFO(get_logger(),
                "Grid: v in [%.2f, %.2f], w in [-%.2f, %.2f], "
                "n_v=%d, n_w=%d, cells=%zu, cycle=%.1f s (%.1f min)",
                v_min_ref_, v_max_ref_,
                w_max_ref_, w_max_ref_,
                n_v_, n_w_, total_cells,
                total_cycle_time_sec_, total_cycle_time_sec_ / 60.0);
  }

  void onTimerTick() {
    if (!has_odom_ || inputs_.empty()) {
      return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    const double t_cell =
        std::chrono::duration<double>(now_steady - last_switch_time_).count();

    // 셀 변경 시: 인덱스 갱신 + 현재 셀 목표 v,w 및 진행 상황 출력
    if (t_cell >= dwell_time_sec_) {
      current_index_ = (current_index_ + 1) % inputs_.size();
      last_switch_time_ = now_steady;

      if (total_cycle_time_sec_ > 0.0) {
        const double time_in_cycle =
            dwell_time_sec_ * static_cast<double>(current_index_);
        const double time_remaining =
            std::max(0.0, total_cycle_time_sec_ - time_in_cycle);
        const double progress_ratio = time_in_cycle / total_cycle_time_sec_;

        const GridCellCommand &cur_cell = inputs_[current_index_];

        RCLCPP_INFO(
            get_logger(),
            "Switch to cell %zu/%zu: v_target=%.2f m/s, w_target=%.2f rad/s, "
            "cycle=%.1f%%, remaining=%.1f s",
            current_index_ + 1,
            inputs_.size(),
            cur_cell.v_bias,
            cur_cell.w_bias,
            progress_ratio * 100.0,
            time_remaining);
      }

      return;
    }

    const GridCellCommand &cur = inputs_[current_index_];
    constexpr double PI = 3.14159265358979323846;

    // v_ref(t) = v_bias + sine sweep (per-cell amplitude)
    const double f_v    = computeSweepFrequency(
        f_v_start_, f_v_end_, dwell_time_sec_, t_cell);
    const double v_sine = cur.v_amp * std::sin(2.0 * PI * f_v * t_cell);
    double v_ref        = cur.v_bias + v_sine;
    v_ref = clamp(v_ref, v_min_ref_, v_max_ref_);  // 안전용

    // w_ref(t) = w_bias + sine sweep (per-cell amplitude)
    const double f_w    = computeSweepFrequency(
        f_w_start_, f_w_end_, dwell_time_sec_, t_cell);
    const double w_sine = cur.w_amp * std::sin(2.0 * PI * f_w * t_cell);
    double w_ref        = cur.w_bias + w_sine;
    w_ref = clamp(w_ref, -w_max_ref_, w_max_ref_);

    // Measured v, w
    const auto &tw_lin  = odom_.twist.twist.linear;
    const auto &tw_ang  = odom_.twist.twist.angular;
    const double v_meas = tw_lin.x;
    const double w_meas = tw_ang.z;

    // Longitudinal control
    double a_cmd = k_v_ * (v_ref - v_meas);

    // Yaw-rate control (FF + FB)
    constexpr double v_eps = 0.1;
    double v_for_curv = (std::fabs(v_meas) > v_eps) ? v_meas : v_ref;
    if (std::fabs(v_for_curv) < v_eps) {
      v_for_curv = (v_ref >= 0.0) ? v_eps : -v_eps;
    }

    double delta_ff  = std::atan(wheelbase_ * w_ref / v_for_curv);
    double delta_fb  = k_w_ * (w_ref - w_meas);
    double steer_cmd = delta_ff + delta_fb;

    steer_cmd = clamp(steer_cmd, -steer_max_, steer_max_);
    a_cmd     = clamp(a_cmd,     a_min_,      a_max_);

    publishDriveCommand(steer_cmd, a_cmd);
  }

  void publishDriveCommand(double steering_angle, double acceleration) {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steering_angle;
    cmd.drive.acceleration   = acceleration;
    drive_pub_->publish(cmd);
  }

  // ROS
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry odom_;
  bool has_odom_{false};

  // Limits / ranges
  double wheelbase_{0.0};
  double a_min_{0.0}, a_max_{0.0};
  double steer_max_{0.0};
  double v_min_ref_{0.0}, v_max_ref_{0.0};
  double w_max_ref_{0.0};

  // Grid
  int n_v_{0};
  int n_w_{0};
  double dwell_time_sec_{0.0};
  bool shuffle_grid_{false};
  std::vector<GridCellCommand> inputs_;
  std::size_t current_index_{0};
  double total_cycle_time_sec_{0.0};

  // Sine sweep (global max)
  double A_v_max_{0.0}, f_v_start_{0.0}, f_v_end_{0.0};
  double A_w_max_{0.0}, f_w_start_{0.0}, f_w_end_{0.0};

  // Gains
  double k_v_{0.0};
  double k_w_{0.0};

  // Misc
  std::string odom_topic_;
  std::string drive_topic_;
  std::chrono::steady_clock::time_point last_switch_time_;
  std::mt19937 rng_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridSweepExcitationNode>());
  rclcpp::shutdown();
  return 0;
}
