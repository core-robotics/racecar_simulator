// ROS2 lap stats node with robust S/F crossing and safe debounce
// - Starts timing after first valid S/F crossing
// - Direction inferred online; bidirectional handling
// - Debounce uses lap-start time, and last_cross_time updates only on accepted crossings

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <chrono>
#include <cmath>
#include <sstream>
#include <limits>
#include <vector>

class RaceStatsNode : public rclcpp::Node {
public:
  RaceStatsNode() : Node("race_stats_node") {
    // Parameters
    odom_topic_           = declare_parameter<std::string>("odom_topic", "odom0");
    collision_topic_      = declare_parameter<std::string>("collision_topic", "collision0");
    path_topic_           = declare_parameter<std::string>("path_topic", "center_path");
    text_frame_           = declare_parameter<std::string>("text_frame", "map");
    fixed_frame_          = declare_parameter<std::string>("fixed_frame", "map");
    text_anchor_x_        = declare_parameter<double>("text_anchor_x", 0.0);
    text_anchor_y_        = declare_parameter<double>("text_anchor_y", 0.0);
    text_scale_           = declare_parameter<double>("text_scale", 0.25);
    min_lap_time_         = declare_parameter<double>("min_lap_time", 3.0);
    update_rate_hz_       = declare_parameter<double>("update_rate_hz", 10.0);
    start_on_first_cross_ = declare_parameter<bool>("start_on_first_cross", true);
    one_sided_forward_    = declare_parameter<bool>("one_sided_forward", true);
    cross_eps_            = declare_parameter<double>("sf_cross_epsilon", 0.15);

    // QoS
    auto r_qos   = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto b_qos   = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto r_t_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    // Subs
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, b_qos, std::bind(&RaceStatsNode::onOdom, this, std::placeholders::_1));
    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
        collision_topic_, b_qos, std::bind(&RaceStatsNode::onCollision, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, r_t_qos, std::bind(&RaceStatsNode::onCenterPath, this, std::placeholders::_1));

    // Pubs
    lap_cnt_pub_       = create_publisher<std_msgs::msg::Int32>("/lap_count", r_qos);
    cur_lap_time_pub_  = create_publisher<std_msgs::msg::Float64>("/current_lap_time", r_qos);
    last_lap_time_pub_ = create_publisher<std_msgs::msg::Float64>("/last_lap_time", r_qos);
    best_lap_time_pub_ = create_publisher<std_msgs::msg::Float64>("/best_lap_time", r_qos);
    collision_cnt_pub_ = create_publisher<std_msgs::msg::Int32>("/collision_count", r_qos);
    stats_str_pub_     = create_publisher<std_msgs::msg::String>("/race_stats", r_qos);
    marker_pub_        = create_publisher<visualization_msgs::msg::MarkerArray>("/race_stats_markers", r_t_qos);

    // Timer
    auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(period, std::bind(&RaceStatsNode::onTimer, this));
  }

private:
  // Core state
  std::string odom_topic_, collision_topic_, path_topic_, text_frame_, fixed_frame_;
  double text_anchor_x_{0.0}, text_anchor_y_{0.0}, text_scale_{0.25};
  double min_lap_time_{3.0};
  double update_rate_hz_{10.0};
  bool start_on_first_cross_{true};
  bool one_sided_forward_{true};
  double cross_eps_{0.15};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lap_cnt_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cur_lap_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr last_lap_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr best_lap_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr collision_cnt_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_str_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Path / projection
  std::vector<double> px_, py_;
  std::vector<double> seg_dx_, seg_dy_, seg_len_;
  std::vector<double> s_node_;
  double track_len_{0.0};
  bool have_path_{false};
  int last_seg_hint_{0};

  // S/F line params (from node 0->1)
  bool line_published_{false};
  double p0x_{0.0}, p0y_{0.0};
  double tx_{0.0}, ty_{1.0};   // line direction (perp to tangent)
  double n0x_{1.0}, n0y_{0.0}; // line normal (path tangent at node 0)

  // Stats
  int lap_count_{0};
  int collision_count_{0};
  bool prev_collision_{false};

  rclcpp::Time lap_start_time_{};
  rclcpp::Time last_cross_time_{};
  bool running_lap_{false};
  double last_lap_time_sec_{0.0};
  double best_lap_time_sec_{std::numeric_limits<double>::infinity()};
  double current_lap_time_sec_{0.0};

  // Gate thresholds
  double gate_low_{0.0}, gate_high_{0.0};

  // s history
  bool have_prev_s_{false};
  double prev_s_{0.0};

  // signed-distance history
  bool have_prev_sd_{false};
  double prev_sd_{0.0};

  // online direction estimate (+1 increasing s, -1 decreasing s)
  double ds_ema_{0.0};
  int dir_sign_{+1};

  // HUD text dedup
  std::string last_text_;

  // Helpers
  static double dot(double ax, double ay, double bx, double by) { return ax*bx + ay*by; }
  static double norm(double x, double y) { return std::sqrt(x*x + y*y); }
  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

  void onCenterPath(const nav_msgs::msg::Path::SharedPtr msg) {
    const auto &poses = msg->poses;
    if (poses.size() < 2) return;

    // Load points
    px_.clear(); py_.clear();
    px_.reserve(poses.size()); py_.reserve(poses.size());
    for (const auto &ps : poses) {
      px_.push_back(ps.pose.position.x);
      py_.push_back(ps.pose.position.y);
    }

    // Precompute segments and cumulative s (closed loop)
    const int N = static_cast<int>(px_.size());
    seg_dx_.assign(N, 0.0);
    seg_dy_.assign(N, 0.0);
    seg_len_.assign(N, 0.0);
    s_node_.assign(N+1, 0.0);

    track_len_ = 0.0;
    for (int i = 0; i < N; ++i) {
      int j = (i+1) % N;
      double dx = px_[j]-px_[i];
      double dy = py_[j]-py_[i];
      double L  = norm(dx, dy);
      if (L < 1e-6) L = 1e-6;
      seg_dx_[i] = dx; seg_dy_[i] = dy; seg_len_[i] = L;
      s_node_[i+1] = s_node_[i] + L;
    }
    track_len_ = s_node_[N];

    // S/F line at node 0
    p0x_ = px_[0]; p0y_ = py_[0];
    const double L01 = norm(seg_dx_[0], seg_dy_[0]);
    tx_  = (L01>1e-6) ? -seg_dy_[0]/L01 : 0.0;
    ty_  = (L01>1e-6) ?  seg_dx_[0]/L01 : 1.0;
    n0x_ = (L01>1e-6) ?  seg_dx_[0]/L01 : 1.0;
    n0y_ = (L01>1e-6) ?  seg_dy_[0]/L01 : 0.0;

    // s-gate (backup)
    gate_low_  = 0.2 * track_len_;
    gate_high_ = 0.8 * track_len_;

    have_path_ = true;
    have_prev_s_ = false;
    have_prev_sd_ = false;
    line_published_ = false;
    last_seg_hint_ = 0;

    // reset direction estimate
    ds_ema_ = 0.0;
    dir_sign_ = +1;
  }

  // Project (x,y) to arc-length s in [0, L)
  double projectToPath(double x, double y) {
    if (!have_path_) return 0.0;
    const int N = static_cast<int>(px_.size());
    if (N < 2) return 0.0;

    int best_i = -1; double best_d2 = std::numeric_limits<double>::infinity();
    auto try_seg = [&](int i){
      const double vx = seg_dx_[i], vy = seg_dy_[i];
      const double wx = x - px_[i],  wy = y - py_[i];
      const double t  = clamp(dot(wx, wy, vx, vy) / (seg_len_[i]*seg_len_[i]), 0.0, 1.0);
      const double projx = px_[i] + t*vx;
      const double projy = py_[i] + t*vy;
      const double dx = x - projx, dy = y - projy;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best_i = i; }
    };

    const int W = std::min(20, N);
    for (int k = -W; k <= W; ++k) {
      int i = (last_seg_hint_ + k) % N; if (i<0) i += N;
      try_seg(i);
    }
    if (best_i < 0) {
      for (int i=0;i<N;++i) try_seg(i);
    }
    last_seg_hint_ = best_i;

    const double vx = seg_dx_[best_i], vy = seg_dy_[best_i];
    const double wx = x - px_[best_i],  wy = y - py_[best_i];
    const double t  = clamp(dot(wx, wy, vx, vy) / (seg_len_[best_i]*seg_len_[best_i]), 0.0, 1.0);
    const double s  = s_node_[best_i] + t*seg_len_[best_i];
    return (s >= track_len_) ? (s - track_len_) : s;
  }

  // Signed distance to S/F line (positive in +tangent direction)
  inline double signedDistSF(double x, double y) const {
    return (x - p0x_) * n0x_ + (y - p0y_) * n0y_;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    if (running_lap_) {
      current_lap_time_sec_ = (this->get_clock()->now() - lap_start_time_).seconds();
    }
    if (!have_path_) return;

    const double s  = projectToPath(x, y);
    const double sd = signedDistSF(x, y);

    // First samples
    if (!have_prev_s_)  { prev_s_ = s;  have_prev_s_  = true; }
    if (!have_prev_sd_) { prev_sd_ = sd; have_prev_sd_ = true; }

    // Wrap-aware ds for direction
    double ds_raw = s - prev_s_;
    if (ds_raw >  0.5 * track_len_) ds_raw -= track_len_;
    if (ds_raw < -0.5 * track_len_) ds_raw += track_len_;

    // EMA for direction estimate
    ds_ema_ = 0.9 * ds_ema_ + 0.1 * ds_raw;
    dir_sign_ = (ds_ema_ >= 0.0) ? +1 : -1;

    // s-gate crossings (both ways)
    const bool gate_inc = (prev_s_ > gate_high_) && (s < gate_low_);
    const bool gate_dec = (prev_s_ < gate_low_)  && (s > gate_high_);

    // wrap crossings (both ways)
    const bool wrap_inc = (s - prev_s_) < -0.5 * track_len_;
    const bool wrap_dec = (s - prev_s_) >  +0.5 * track_len_;

    // signed-distance with hysteresis; forward depends on dir_sign_
    bool sd_fwd = false, sd_bwd = false;
    if (dir_sign_ >= 0) {
      sd_fwd = (prev_sd_ <= -cross_eps_) && (sd >=  cross_eps_);
      sd_bwd = (prev_sd_ >=  cross_eps_) && (sd <= -cross_eps_);
    } else { // reverse driving flips roles
      sd_fwd = (prev_sd_ >=  cross_eps_) && (sd <= -cross_eps_);
      sd_bwd = (prev_sd_ <= -cross_eps_) && (sd >=  cross_eps_);
    }

    const bool crossed_gate = gate_inc || gate_dec;
    const bool crossed_wrap = wrap_inc || wrap_dec;
    const bool crossed_line = one_sided_forward_ ? sd_fwd : (sd_fwd || sd_bwd);

    const bool crossed = crossed_gate || crossed_wrap || crossed_line;

    if (crossed) {
      const rclcpp::Time now_t = this->get_clock()->now();

      // Debounce by lap-start, not by last-cross (prevents “fake early cross” from blocking the next real one)
      const double since_lap_start = lap_start_time_.nanoseconds() > 0
                                     ? (now_t - lap_start_time_).seconds()
                                     : std::numeric_limits<double>::infinity();

      bool accepted = false;
      if (running_lap_) {
        if (since_lap_start > min_lap_time_) {
          const double lap_time = (now_t - lap_start_time_).seconds();
          last_lap_time_sec_ = lap_time;
          if (lap_time < best_lap_time_sec_) best_lap_time_sec_ = lap_time;
          lap_count_++;
          lap_start_time_ = now_t;
          accepted = true;
        }
      } else if (start_on_first_cross_) {
        running_lap_ = true;
        lap_start_time_ = now_t;
        accepted = true;
      }

      // Only update last_cross_time_ when a crossing is accepted
      if (accepted) {
        last_cross_time_ = now_t;
      }
    }

    prev_s_  = s;
    prev_sd_ = sd;
  }

  void onCollision(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool coll = msg->data;
    if (coll && !prev_collision_) {
      collision_count_++;
    }
    prev_collision_ = coll;
  }

  void onTimer() {
    publishNumericTopics();
    publishMarkers();
    publishStringStats();
  }

  void publishNumericTopics() {
    std_msgs::msg::Int32 i32;
    std_msgs::msg::Float64 f64;

    i32.data = lap_count_;           lap_cnt_pub_->publish(i32);
    i32.data = collision_count_;     collision_cnt_pub_->publish(i32);

    f64.data = running_lap_ ? current_lap_time_sec_ : 0.0;  cur_lap_time_pub_->publish(f64);
    f64.data = last_lap_time_sec_;                          last_lap_time_pub_->publish(f64);
    f64.data = std::isfinite(best_lap_time_sec_) ? best_lap_time_sec_ : 0.0; best_lap_time_pub_->publish(f64);
  }

  void publishStringStats() {
    std_msgs::msg::String s;
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "laps=" << lap_count_
        << ", cur_lap=" << (running_lap_ ? current_lap_time_sec_ : 0.0) << "s"
        << ", last_lap=" << last_lap_time_sec_ << "s"
        << ", best_lap=" << (std::isfinite(best_lap_time_sec_) ? best_lap_time_sec_ : 0.0) << "s"
        << ", collisions=" << collision_count_;
    s.data = oss.str();
    stats_str_pub_->publish(s);
  }

  void publishMarkers() {
    builtin_interfaces::msg::Time stamp{};
    visualization_msgs::msg::MarkerArray arr;

    // HUD text
    std::ostringstream oss; oss.setf(std::ios::fixed); oss.precision(3);
    oss << "Lap: " << lap_count_ << "\n"
        << "Current: " << (running_lap_ ? current_lap_time_sec_ : 0.0) << " s\n"
        << "Last:    " << last_lap_time_sec_ << " s\n"
        << "Best:    " << (std::isfinite(best_lap_time_sec_) ? best_lap_time_sec_ : 0.0) << " s\n"
        << "Collisions: " << collision_count_;
    std::string new_text = oss.str();

    if (new_text == last_text_ && (have_path_ ? line_published_ : true)) return;

    visualization_msgs::msg::Marker text;
    text.header.frame_id = text_frame_;
    text.header.stamp = stamp;
    text.ns = "race_stats"; text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = text_anchor_x_;
    text.pose.position.y = text_anchor_y_;
    text.pose.position.z = 0.5;
    text.pose.orientation.w = 1.0;
    text.scale.z = text_scale_;
    text.color.a = 1.0; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0;
    text.lifetime = rclcpp::Duration(0, 0);
    text.text = new_text;
    arr.markers.push_back(text);
    last_text_ = new_text;

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
      line.lifetime = rclcpp::Duration(0, 0);

      geometry_msgs::msg::Point pA, pB;
      const double half_len = 1.0;
      pA.x = p0x_ - half_len * tx_; pA.y = p0y_ - half_len * ty_; pA.z = 0.05;
      pB.x = p0x_ + half_len * tx_; pB.y = p0y_ + half_len * ty_; pB.z = 0.05;
      line.points.push_back(pA);
      line.points.push_back(pB);

      arr.markers.push_back(line);
      line_published_ = true;
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
