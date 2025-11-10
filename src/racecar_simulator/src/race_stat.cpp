// ROS2 Lap Timer Node (Marker-only version)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <limits>
#include <vector>
#include <deque>
#include <sstream>

class RaceStatsNode : public rclcpp::Node {
public:
  RaceStatsNode() : Node("race_stats_node") {
    // Parameters
    odom_topic_ = declare_parameter("odom_topic", "odom0");
    collision_topic_ = declare_parameter("collision_topic", "collision0");
    path_topic_ = declare_parameter("path_topic", "center_path");
    text_frame_ = declare_parameter("text_frame", "base_link0");
    fixed_frame_ = declare_parameter("fixed_frame", "map");
    text_anchor_x_ = declare_parameter("text_anchor_x", 0.0);
    text_anchor_y_ = declare_parameter("text_anchor_y", 0.0);
    text_scale_ = declare_parameter("text_scale", 0.3);
    min_lap_time_ = declare_parameter("min_lap_time", 3.0);
    update_rate_hz_ = declare_parameter("update_rate_hz", 10.0);
    start_on_first_cross_ = declare_parameter("start_on_first_cross", true);

    // QoS
    auto r_t_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto b_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Subs
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, b_qos, std::bind(&RaceStatsNode::onOdom, this, std::placeholders::_1));
    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
        collision_topic_, b_qos, std::bind(&RaceStatsNode::onCollision, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, r_t_qos, std::bind(&RaceStatsNode::onCenterPath, this, std::placeholders::_1));

    // Marker publisher
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/race_stats_markers", r_t_qos);

    // Timer
    auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(period, std::bind(&RaceStatsNode::onTimer, this));
  }

private:
  // Params
  std::string odom_topic_, collision_topic_, path_topic_, text_frame_, fixed_frame_;
  double text_anchor_x_, text_anchor_y_, text_scale_, min_lap_time_, update_rate_hz_;
  bool start_on_first_cross_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Path
  std::vector<double> px_, py_, seg_dx_, seg_dy_, seg_len_, s_node_;
  double track_len_{0.0};
  bool have_path_{false};
  int last_seg_hint_{0};

  // Gate
  double gate_low_{0.0}, gate_high_{0.0};
  double p0x_{0.0}, p0y_{0.0}, tx_{0.0}, ty_{1.0};

  // Lap
  int lap_count_{0}, collision_count_{0};
  bool running_lap_{false}, prev_collision_{false}, have_prev_s_{false};
  double prev_s_{0.0};
  rclcpp::Time lap_start_time_{}, last_cross_time_{};
  double last_lap_time_sec_{0.0}, best_lap_time_sec_{std::numeric_limits<double>::infinity()}, current_lap_time_sec_{0.0};
  std::deque<double> lap_hist_;

  // Utils
  static double dot(double ax, double ay, double bx, double by) { return ax * bx + ay * by; }
  static double norm(double x, double y) { return std::sqrt(x * x + y * y); }
  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

  void onCenterPath(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.size() < 2) return;

    // Load points
    px_.clear(); py_.clear();
    for (auto &p : msg->poses) {
      px_.push_back(p.pose.position.x);
      py_.push_back(p.pose.position.y);
    }

    // Close loop if endpoints apart
    double dx_end = px_.front() - px_.back();
    double dy_end = py_.front() - py_.back();
    double gap = std::sqrt(dx_end * dx_end + dy_end * dy_end);
    if (gap > 0.05) {
    int n_interp = static_cast<int>(gap / 0.05);
      for (int i = 1; i <= n_interp; ++i) {
        double t = static_cast<double>(i) / (n_interp + 1);
        double xi = px_.back() + t * (px_.front() - px_.back());
        double yi = py_.back() + t * (py_.front() - py_.back());
        px_.push_back(xi);
        py_.push_back(yi);
      }
    }

    // Segments
    int N = px_.size();
    seg_dx_.assign(N, 0.0); seg_dy_.assign(N, 0.0);
    seg_len_.assign(N, 0.0); s_node_.assign(N + 1, 0.0);
    track_len_ = 0.0;
    for (int i = 0; i < N; ++i) {
      int j = (i + 1) % N;
      double dx = px_[j] - px_[i], dy = py_[j] - py_[i];
      double L = norm(dx, dy);
      if (L < 1e-6) L = 1e-6;
      seg_dx_[i] = dx; seg_dy_[i] = dy; seg_len_[i] = L;
      s_node_[i + 1] = s_node_[i] + L;
    }
    track_len_ = s_node_[N];

    // Gate
    p0x_ = px_[0]; p0y_ = py_[0];
    double L01 = norm(seg_dx_[0], seg_dy_[0]);
    tx_ = (L01 > 1e-6) ? -seg_dy_[0] / L01 : 0.0;
    ty_ = (L01 > 1e-6) ? seg_dx_[0] / L01 : 1.0;
    gate_low_ = 0.2 * track_len_;
    gate_high_ = 0.8 * track_len_;

    have_path_ = true;
    have_prev_s_ = false;
  }

  double projectToPath(double x, double y) {
    if (!have_path_) return 0.0;
    int N = px_.size();
    int best_i = -1; double best_d2 = std::numeric_limits<double>::infinity();
    auto try_seg = [&](int i) {
      double vx = seg_dx_[i], vy = seg_dy_[i];
      double wx = x - px_[i], wy = y - py_[i];
      double t = clamp(dot(wx, wy, vx, vy) / (seg_len_[i] * seg_len_[i]), 0.0, 1.0);
      double projx = px_[i] + t * vx, projy = py_[i] + t * vy;
      double d2 = (x - projx) * (x - projx) + (y - projy) * (y - projy);
      if (d2 < best_d2) { best_d2 = d2; best_i = i; }
    };
    for (int k = -10; k <= 10; ++k) {
      int i = (last_seg_hint_ + k + N) % N; try_seg(i);
    }
    last_seg_hint_ = best_i;
    double vx = seg_dx_[best_i], vy = seg_dy_[best_i];
    double wx = x - px_[best_i], wy = y - py_[best_i];
    double t = clamp(dot(wx, wy, vx, vy) / (seg_len_[best_i] * seg_len_[best_i]), 0.0, 1.0);
    double s = s_node_[best_i] + t * seg_len_[best_i];
    return (s >= track_len_) ? (s - track_len_) : s;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!have_path_) return;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    const rclcpp::Time now_t = msg->header.stamp;
    double s = projectToPath(x, y);
    if (!have_prev_s_) { prev_s_ = s; have_prev_s_ = true; return; }

    if (running_lap_) current_lap_time_sec_ = (this->now() - lap_start_time_).seconds();

    double eps = 1e-3 * track_len_;
    bool crossed_gate = (prev_s_ >= gate_high_ - eps) && (s <= gate_low_ + eps);
    double ds = s - prev_s_;
    bool crossed_wrap = (ds < -0.4 * track_len_);
    bool crossed = crossed_gate || crossed_wrap;

    if (crossed) {
      // auto now_t = this->now();
      double since_last = last_cross_time_.nanoseconds() > 0
                            ? (now_t - last_cross_time_).seconds()
                            : std::numeric_limits<double>::infinity();

      if (running_lap_) {
        if (since_last > min_lap_time_) {
          double lap_time = (now_t - lap_start_time_).seconds();
          last_lap_time_sec_ = lap_time;
          best_lap_time_sec_ = std::min(best_lap_time_sec_, lap_time);
          lap_count_++;
          lap_hist_.push_front(lap_time);
          if (lap_hist_.size() > 3) lap_hist_.pop_back();
          lap_start_time_ = now_t;
        }
      } else if (start_on_first_cross_) {
        running_lap_ = true;
        lap_start_time_ = now_t;
      }
      last_cross_time_ = now_t;
    }

    prev_s_ = s;
  }

  void onCollision(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !prev_collision_) collision_count_++;
    prev_collision_ = msg->data;
  }

  void onTimer() {
    publishMarkers();
  }

  void publishMarkers() {
    visualization_msgs::msg::MarkerArray arr;
    builtin_interfaces::msg::Time stamp = now();

    // HUD text
    std::ostringstream oss;
    oss.setf(std::ios::fixed); oss.precision(3);
    oss << "Lap: " << lap_count_ << "\n"
        << "Cur: " << (running_lap_ ? current_lap_time_sec_ : 0.0) << " s\n"
        << "Last: " << last_lap_time_sec_ << " s\n"
        << "Best: " << (std::isfinite(best_lap_time_sec_) ? best_lap_time_sec_ : 0.0) << " s\n"
        << "Col: " << collision_count_;
    if (!lap_hist_.empty()) {
      oss << "\n";
      for (size_t i = 0; i < lap_hist_.size(); ++i)
        oss << "lap" << i+1 << ": " << lap_hist_[i] << " s\n";
    }

    visualization_msgs::msg::Marker text;
    text.header.frame_id = text_frame_;
    text.header.stamp = stamp;
    text.ns = "race_stats"; text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = text_anchor_x_;
    text.pose.position.y = text_anchor_y_;
    text.pose.position.z = 0.5;
    text.scale.z = text_scale_;
    text.color.a = 1.0;
    text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0;
    text.text = oss.str();
    arr.markers.push_back(text);

    // S/F line
    if (have_path_) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = fixed_frame_;
      line.header.stamp = stamp;
      line.ns = "race_stats"; line.id = 2;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.03;
      line.color.a = 1.0; line.color.r = 1.0; line.color.g = 0.2; line.color.b = 0.2;
      geometry_msgs::msg::Point pA, pB;
      const double half_len = 1.0;
      pA.x = p0x_ - half_len * tx_; pA.y = p0y_ - half_len * ty_;
      pB.x = p0x_ + half_len * tx_; pB.y = p0y_ + half_len * ty_;
      line.points.push_back(pA); line.points.push_back(pB);
      arr.markers.push_back(line);
    }

    marker_pub_->publish(arr);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaceStatsNode>());
  rclcpp::shutdown();
  return 0;
}
