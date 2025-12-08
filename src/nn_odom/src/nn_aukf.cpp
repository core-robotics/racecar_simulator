// src/nn_aukf_node.cpp

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <deque>
#include <array>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

#include <torch/script.h>
#include <torch/torch.h>

class NNAukfNode : public rclcpp::Node
{
public:
  NNAukfNode()
  : Node("nn_aukf")
  {
    basic_ukf_path_ = declare_parameter<std::string>("basic_ukf_path", "");
    nn_ukf_path_    = declare_parameter<std::string>("nn_ukf_path", "");
    use_gpu_        = declare_parameter<bool>("use_gpu", true);
    mu_init_        = declare_parameter<double>("mu_init", 1.0);

    double hz = declare_parameter<double>("timer_period", 100.0);
    double period_sec = 1.0 / hz;

    odom_topic_          = declare_parameter<std::string>("odom_topic", "/odom");
    imu_topic_           = declare_parameter<std::string>("imu_topic", "/imu/data");
    control_topic_       = declare_parameter<std::string>("control_topic", "/ackermann_cmd");
    motor_current_topic_ = declare_parameter<std::string>("motor_current_topic", "/sensors/core");

    device_ = (use_gpu_ && torch::cuda::is_available())
                ? torch::Device(torch::kCUDA)
                : torch::Device(torch::kCPU);

    load_models();
    init_filter();
    setup_subscribers();

    odom_nn_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_nn", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&NNAukfNode::timer_callback, this)
    );
  }

private:
  void setup_subscribers()
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      std::bind(&NNAukfNode::odom_callback, this, std::placeholders::_1)
    );

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&NNAukfNode::imu_callback, this, std::placeholders::_1)
    );

    control_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      control_topic_, 10,
      std::bind(&NNAukfNode::control_callback, this, std::placeholders::_1)
    );

    motor_current_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
      motor_current_topic_, sensor_qos,
      std::bind(&NNAukfNode::motor_current_callback, this, std::placeholders::_1)
    );
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    omega_wheels_ = msg->twist.twist.linear.x;
    got_odom_ = true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    ax_imu_ = msg->linear_acceleration.x;
    ay_imu_ = msg->linear_acceleration.y;
    r_imu_  = msg->angular_velocity.z;
    got_imu_ = true;
  }

  void control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    delta_ = msg->drive.steering_angle;
    got_control_ = true;
  }

  void motor_current_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    Iq_ = msg->state.current_motor;
    got_motor_ = true;
  }

  void load_models()
  {
    basic_ukf_module_ = torch::jit::load(basic_ukf_path_);
    nn_module_        = torch::jit::load(nn_ukf_path_);

    basic_ukf_module_.eval();
    nn_module_.eval();

    basic_ukf_module_.to(device_);
    nn_module_.to(device_);
  }

  void init_filter()
  {
    auto opts = torch::TensorOptions().dtype(torch::kFloat32).device(device_);

    x_ = torch::zeros({1, 5}, opts);
    x_.index_put_({0, 4}, static_cast<float>(mu_init_));

    torch::Tensor p_diag = torch::tensor({1e-2f, 1e-2f, 1e-2f, 1e-2f, 1e-3f}, opts);
    P_ = p_diag.diag_embed().unsqueeze(0);

    R_diag_prev_ = torch::tensor({{50.0f, 50.0f, 0.1f, 0.1f}}, opts);

    innov_state_hist_.clear();
    std::array<float, 9> zero{};
    zero.fill(0.0f);
    for (int i = 0; i < 30; ++i)
      innov_state_hist_.push_back(zero);

    step_count_ = 0;
  }

  torch::Tensor build_u(float delta, float Iq)
  {
    auto opts = torch::TensorOptions().dtype(torch::kFloat32).device(device_);
    return torch::tensor({{delta, Iq}}, opts);
  }

  torch::Tensor build_z(float ax, float ay, float r_imu, float omega_wheels)
  {
    auto opts = torch::TensorOptions().dtype(torch::kFloat32).device(device_);
    return torch::tensor({{ax, ay, r_imu, omega_wheels}}, opts);
  }

  torch::Tensor build_innov_hist()
  {
    auto opts_cpu = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU);
    torch::Tensor hist_cpu = torch::zeros({30, 9}, opts_cpu);

    int i = 0;
    for (const auto &v : innov_state_hist_)
    {
      for (int j = 0; j < 9; ++j)
        hist_cpu[i][j] = v[j];
      if (++i >= 30) break;
    }

    return hist_cpu.to(device_).unsqueeze(0);
  }

  void push_innov_state(const torch::Tensor &innov, const torch::Tensor &x_hat)
  {
    auto innov_cpu = innov.to(torch::kCPU).contiguous();
    auto x_cpu = x_hat.to(torch::kCPU).contiguous();

    std::array<float, 9> v{};
    for (int i = 0; i < 4; ++i)
      v[i] = innov_cpu[0][i].item<float>();
    for (int i = 0; i < 5; ++i)
      v[4 + i] = x_cpu[0][i].item<float>();

    if ((int)innov_state_hist_.size() >= 30)
      innov_state_hist_.pop_front();
    innov_state_hist_.push_back(v);
  }

  void publish_odom_nn()
  {
    auto x_cpu = x_.to(torch::kCPU).contiguous();

    float vx = x_cpu[0][0].item<float>();
    float vy = x_cpu[0][1].item<float>();
    float r  = x_cpu[0][2].item<float>();
    float omega_wheels = x_cpu[0][3].item<float>();
    float mu = x_cpu[0][4].item<float>();

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";

    // NOTE: 기존 코드 유지
    msg.twist.twist.linear.x  = vx;
    msg.twist.twist.linear.y  = vy;
    msg.twist.twist.angular.z = r;
    msg.twist.twist.linear.z  = omega_wheels;
    msg.twist.twist.angular.x = mu;

    odom_nn_pub_->publish(msg);
  }

  void timer_callback()
  {
    if (!(got_odom_ && got_imu_ && got_control_ && got_motor_))
      return;

    float omega_wheels, delta, Iq, ax_imu, ay_imu, r_imu;
    {
      std::lock_guard<std::mutex> lock(data_mtx_);
      omega_wheels = (float)omega_wheels_;
      delta        = (float)delta_;
      Iq           = (float)Iq_;
      ax_imu       = (float)ax_imu_;
      ay_imu       = (float)ay_imu_;
      r_imu        = (float)r_imu_;
    }

    torch::Tensor u = build_u(delta, Iq);
    torch::Tensor z = build_z(ax_imu, ay_imu, r_imu, omega_wheels);

    torch::NoGradGuard no_grad;

    // ---------------------------
    // Warmup: basic UKF 30 steps
    // ---------------------------
    if (step_count_ < 30)
    {
      std::vector<torch::jit::IValue> inputs;
      inputs.reserve(4);
      inputs.push_back(x_);
      inputs.push_back(u);
      inputs.push_back(z);
      inputs.push_back(P_);

      auto out = basic_ukf_module_.forward(inputs).toTuple();

      torch::Tensor x_hat = out->elements()[0].toTensor();
      torch::Tensor P_hat = out->elements()[1].toTensor();
      torch::Tensor innov = out->elements()[2].toTensor();

      push_innov_state(innov, x_hat);

      x_ = x_hat;
      P_ = P_hat;

      step_count_++;

      // ✅ 웜업 종료 메시지: 30회 완료 직후 1회 출력
      if (step_count_ == 30)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Basic UKF warmup completed (30 steps). Switching to NN-UKF."
        );
      }

      publish_odom_nn();
      return;
    }

    // ---------------------------
    // NN-UKF
    // ---------------------------
    torch::Tensor innov_hist = build_innov_hist();

    std::vector<torch::jit::IValue> inputs;
    inputs.reserve(6);
    inputs.push_back(x_);
    inputs.push_back(u);
    inputs.push_back(z);
    inputs.push_back(P_);
    inputs.push_back(innov_hist);
    inputs.push_back(R_diag_prev_);

    // ✅ NN-UKF 추론 시간 매번 출력
    auto t0 = std::chrono::steady_clock::now();
    auto out = nn_module_.forward(inputs).toTuple();
    auto t1 = std::chrono::steady_clock::now();

    auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    RCLCPP_INFO(
      this->get_logger(),
      "NN-UKF inference time: %ld us (%.3f ms)",
      (long)us, us / 1000.0
    );

    torch::Tensor x_hat  = out->elements()[0].toTensor();
    torch::Tensor P_hat  = out->elements()[1].toTensor();
    torch::Tensor innov  = out->elements()[2].toTensor();
    torch::Tensor R_diag = out->elements()[3].toTensor();

    push_innov_state(innov, x_hat);

    x_ = x_hat;
    P_ = P_hat;
    R_diag_prev_ = R_diag;

    step_count_++;

    publish_odom_nn();
  }

private:
  std::string basic_ukf_path_;
  std::string nn_ukf_path_;

  std::string odom_topic_;
  std::string imu_topic_;
  std::string control_topic_;
  std::string motor_current_topic_;

  bool use_gpu_{false};
  torch::Device device_{torch::kCPU};

  double mu_init_{1.0};

  torch::jit::script::Module basic_ukf_module_;
  torch::jit::script::Module nn_module_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr motor_current_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_nn_pub_;

  std::mutex data_mtx_;
  double omega_wheels_{0.0};
  double delta_{0.0};
  double Iq_{0.0};
  double ax_imu_{0.0};
  double ay_imu_{0.0};
  double r_imu_{0.0};

  bool got_odom_{false};
  bool got_imu_{false};
  bool got_control_{false};
  bool got_motor_{false};

  torch::Tensor x_;
  torch::Tensor P_;
  torch::Tensor R_diag_prev_;

  std::deque<std::array<float, 9>> innov_state_hist_;

  int64_t step_count_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NNAukfNode>());
  rclcpp::shutdown();
  return 0;
}
