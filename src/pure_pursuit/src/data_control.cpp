#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>  
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <random>
#include <optional>
#include <cmath>
#include <algorithm>
#include <string>
#include <initializer_list>

struct P2 { double x=0.0, y=0.0; };
static inline double sqr(double v){ return v*v; }
static inline double dist2(const P2& a, const P2& b){ return sqr(a.x-b.x)+sqr(a.y-b.y); }

static inline double yawFromQuat(const geometry_msgs::msg::Quaternion& q){
  tf2::Quaternion tq(q.x,q.y,q.z,q.w);
  tf2::Matrix3x3 m(tq);
  double r,p,y; m.getRPY(r,p,y);
  return y;
}

static inline P2 projSeg(const P2& p, const P2& a, const P2& b, double& t01){
  const double vx=b.x-a.x, vy=b.y-a.y;
  const double wx=p.x-a.x, wy=p.y-a.y;
  const double denom = vx*vx + vy*vy;
  double t = (denom>1e-12) ? (wx*vx + wy*vy)/denom : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  t01 = t;
  return {a.x + t*vx, a.y + t*vy};
}

class PathDriverNode final : public rclcpp::Node {
public:
  PathDriverNode() : rclcpp::Node("path_driver_node") {
    pose_topic_  = declare_parameter<std::string>("pose_topic",  "mcl_pose");
    odom_topic_  = declare_parameter<std::string>("odom_topic",  "odom");
    drive_topic_ = declare_parameter<std::string>("drive_topic", "ackermann_cmd");
    path_topic_  = declare_parameter<std::string>("path_topic",  "active_path");
    frame_id_    = declare_parameter<std::string>("frame_id",    "map");

    margin_        = declare_parameter<double>("margin", 0.50);
    samples_       = declare_parameter<int>("samples", 160);
    wheelbase_     = declare_parameter<double>("wheelbase", 0.33);
    switch_radius_ = declare_parameter<double>("switch_radius", 0.5);

    la_base_ = declare_parameter<double>("lookahead_base", 0.6);
    la_gain_ = declare_parameter<double>("lookahead_gain", 0.5);
    la_min_  = declare_parameter<double>("lookahead_min",  0.7);
    la_max_  = declare_parameter<double>("lookahead_max",  2.5);

    vmin_ = declare_parameter<double>("speed_min", 1.0);
    vmax_ = declare_parameter<double>("speed_max", 4.0);
    tmin_ = declare_parameter<double>("change_min_s", 0.2);
    tmax_ = declare_parameter<double>("change_max_s", 4.0);
    if (vmin_ > vmax_) std::swap(vmin_, vmax_);
    if (tmin_ > tmax_) std::swap(tmin_, tmax_);
    tmin_ = std::max(1e-3, tmin_);
    tmax_ = std::max(tmin_, tmax_);

    // Track bounds (keep as your original)
    x_min_ = -2.0; x_max_ =  2.0;
    y_min_ = -9.0; y_max_ = -0.5;


    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){
        last_pose_ = *msg;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg){
        last_odom_ = *msg;
      });

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    path_pub_  = create_publisher<nav_msgs::msg::Path>(path_topic_, 1);

    rng_.seed(std::random_device{}());
    buildTemplates();

    // Randomize direction each segment
    reverse_dir_ = coin();
    start_at_top_ = true;
    selectNextPath();

    rescheduleSpeed();
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]{ tick(); });
  }

private:
  enum class Tid { CapsuleR, CapsuleL, EightS1, EightS2 };

  double urand(double a, double b){
    std::uniform_real_distribution<double> d(a,b);
    return d(rng_);
  }
  bool coin(){
    std::uniform_int_distribution<int> d(0,1);
    return d(rng_)==1;
  }

  void rescheduleSpeed(){
    target_v_ = urand(vmin_, vmax_);
    const double dt = urand(tmin_, tmax_);
    next_v_change_ = now() + rclcpp::Duration::from_seconds(dt);
  }

  double lookahead(double v) const {
    const double L = la_base_ + la_gain_ * std::abs(v);
    return std::clamp(L, la_min_, la_max_);
  }

  struct Closest {
    double s=0.0;
    int seg=0;
    double t=0.0;
    P2 q{};
  };

  Closest closestOnPath(const P2& p) const {
    Closest best;
    if (path_.size() < 2) return best;

    double best_d2 = 1e300;
    for (int i=0; i<(int)path_.size()-1; ++i){
      double t01=0.0;
      const P2 q = projSeg(p, path_[i], path_[i+1], t01);
      const double d2 = dist2(p,q);
      if (d2 < best_d2){
        best_d2 = d2;
        best.seg = i;
        best.t = t01;
        best.q = q;
        const double s0 = arc_[i];
        const double s1 = arc_[i+1];
        best.s = s0 + t01*(s1 - s0);
      }
    }
    return best;
  }

  P2 atS(double s) const {
    if (path_.empty()) return {};
    if (path_.size() == 1) return path_.front();

    const double send = arc_.back();
    s = std::clamp(s, 0.0, send);

    auto it = std::upper_bound(arc_.begin(), arc_.end(), s);
    int i = (int)std::max<int>(0, (int)(it - arc_.begin()) - 1);
    i = std::clamp(i, 0, (int)path_.size()-2);

    const double s0 = arc_[i], s1 = arc_[i+1];
    const double t = (s1 > s0) ? (s - s0)/(s1 - s0) : 0.0;
    const auto &a = path_[i], &b = path_[i+1];
    return { a.x + t*(b.x-a.x), a.y + t*(b.y-a.y) };
  }

  void tick(){
    if (!last_pose_ || path_.size() < 2) return;

    if (now() >= next_v_change_) rescheduleSpeed();

    const auto &pp = last_pose_->pose;
    const P2 pos{pp.position.x, pp.position.y};
    const double yaw = yawFromQuat(pp.orientation);

    // Switch by actual current path end (fix)
    const P2 endp = path_.back();
    if (dist2(pos, endp) < sqr(switch_radius_)){
      start_at_top_ = !start_at_top_;
      reverse_dir_ = coin();
      selectNextPath();
      return;
    }

    const auto c = closestOnPath(pos);
    const double v_meas = last_odom_ ? last_odom_->twist.twist.linear.x : 0.0;
    const double Ls = lookahead(v_meas);
    const P2 tgt = atS(c.s + Ls);

    // Pure pursuit in vehicle frame
    const double dx = tgt.x - pos.x;
    const double dy = tgt.y - pos.y;
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double xr =  cy*dx + sy*dy;
    const double yr = -sy*dx + cy*dy;

    const double ld = std::max(1e-3, std::hypot(xr, yr));
    const double kappa = 2.0 * yr / (ld*ld);
    const double steer = std::atan(wheelbase_ * kappa);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_id_;
    cmd.drive.steering_angle = steer;
    cmd.drive.speed = target_v_;
    drive_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "v_cmd=%.2f v=%.2f steer=%.3f la=%.2f", target_v_, v_meas, steer, Ls);
  }

  void selectNextPath(){
    const double yaw = last_pose_ ? yawFromQuat(last_pose_->pose.orientation) : 0.0;
    const Tid capsule = bestForward({Tid::CapsuleR, Tid::CapsuleL}, yaw);
    const Tid eight   = bestForward({Tid::EightS1,  Tid::EightS2},  yaw);
    const Tid chosen  = coin() ? capsule : eight;

    path_ = orient(templatePath(chosen));
    buildArc();
    publishPath();
  }

  Tid bestForward(const std::initializer_list<Tid>& cands, double yaw){
    const double fx = std::cos(yaw), fy = std::sin(yaw);
    Tid best = *cands.begin();
    double best_dot = -1e300;

    for (auto t : cands){
      auto p = orient(templatePath(t));
      if (p.size() < 2) continue;
      const double vx = p[1].x - p[0].x;
      const double vy = p[1].y - p[0].y;
      const double dot = fx*vx + fy*vy;
      if (dot > 0.0) return t;
      if (dot > best_dot){ best_dot = dot; best = t; }
    }
    return best;
  }

  std::vector<P2> orient(std::vector<P2> p) const {
    if (!start_at_top_) std::reverse(p.begin(), p.end());
    if (reverse_dir_)   std::reverse(p.begin(), p.end());
    return p;
  }

  void buildArc(){
    arc_.assign(path_.size(), 0.0);
    for (size_t i=1; i<path_.size(); ++i){
      const auto &a = path_[i-1], &b = path_[i];
      arc_[i] = arc_[i-1] + std::hypot(b.x-a.x, b.y-a.y);
    }
  }

  void publishPath(){
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.poses.reserve(path_.size());
    for (const auto &p : path_){
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
  }

  // ----- Templates -----
  void buildTemplates(){
    const double w = x_max_ - x_min_, h = y_max_ - y_min_;
    const double r_w = 0.5*w - margin_;
    const double r_h = (h - 2.0*margin_) / 4.0;
    radius_ = std::max(0.05, std::min(r_w, r_h));

    const double cy = 0.5*(y_min_ + y_max_);
    top_    = {0.0, cy - 2.0*radius_};
    bottom_ = {0.0, cy + 2.0*radius_};

    cap_r_ = capsule(true,  cy);
    cap_l_ = capsule(false, cy);
    e8_1_  = eight(true,  false, cy);
    e8_2_  = eight(false, true,  cy);

    clamp(cap_r_); clamp(cap_l_); clamp(e8_1_); clamp(e8_2_);
  }

  std::vector<P2> arcPts(double cx, double cy, double a0, double a1, int n) const {
    n = std::max(2, n);
    std::vector<P2> pts; pts.reserve(n);
    for (int i=0; i<n; ++i){
      const double u = double(i)/double(n-1);
      const double a = a0 + (a1-a0)*u;
      pts.push_back({cx + radius_*std::cos(a), cy + radius_*std::sin(a)});
    }
    return pts;
  }

  std::vector<P2> capsule(bool right, double cy){
    const double x  = right ? +radius_ : -radius_;
    const double yt = cy - radius_, yb = cy + radius_;
    const int n_arc  = std::max(2, samples_/4);
    const int n_side = std::max(2, samples_/2);

    auto top_arc = arcPts(0.0, yt, -M_PI/2.0, right ? 0.0 : -M_PI, n_arc);

    std::vector<P2> side; side.reserve(n_side);
    for (int i=0; i<n_side; ++i){
      const double u = double(i)/double(n_side-1);
      side.push_back({x, yt + (yb-yt)*u});
    }

    auto bot_arc = right
      ? arcPts(0.0, yb, 0.0,  M_PI/2.0, n_arc)
      : arcPts(0.0, yb, -M_PI, -3.0*M_PI/2.0, n_arc);

    std::vector<P2> out = top_arc;
    out.insert(out.end(), side.begin()+1, side.end());
    out.insert(out.end(), bot_arc.begin()+1, bot_arc.end());
    return out;
  }

  std::vector<P2> eight(bool top_right, bool bottom_right, double cy){
    const double yt = cy - radius_, yb = cy + radius_;
    const int n = std::max(2, samples_/2);

    auto top_arc = arcPts(0.0, yt, -M_PI/2.0, top_right ? +M_PI/2.0 : -3.0*M_PI/2.0, n);
    auto bot_arc = arcPts(0.0, yb, -M_PI/2.0, bottom_right ? +M_PI/2.0 : -3.0*M_PI/2.0, n);

    top_arc.insert(top_arc.end(), bot_arc.begin()+1, bot_arc.end());
    return top_arc;
  }

  void clamp(std::vector<P2>& pts) const {
    const double xmin = x_min_ + margin_, xmax = x_max_ - margin_;
    const double ymin = y_min_ + margin_, ymax = y_max_ - margin_;
    for (auto &p : pts){
      p.x = std::clamp(p.x, xmin, xmax);
      p.y = std::clamp(p.y, ymin, ymax);
    }
  }

  std::vector<P2> templatePath(Tid t) const {
    switch(t){
      case Tid::CapsuleR: return cap_r_;
      case Tid::CapsuleL: return cap_l_;
      case Tid::EightS1:  return e8_1_;
      case Tid::EightS2:  return e8_2_;
    }
    return cap_r_;
  }

private:
  std::string pose_topic_, odom_topic_, drive_topic_, path_topic_, frame_id_;

  // bounds/templates
  double x_min_=-2.0, x_max_=2.0, y_min_=-9.0, y_max_=-0.5;
  double margin_=0.5;
  int samples_=160;
  double radius_=1.0;
  P2 top_{}, bottom_{};
  std::vector<P2> cap_r_, cap_l_, e8_1_, e8_2_;

  // active path
  std::vector<P2> path_;
  std::vector<double> arc_;

  // state
  std::optional<geometry_msgs::msg::PoseStamped> last_pose_;  
  std::optional<nav_msgs::msg::Odometry> last_odom_;
  bool start_at_top_ = true;
  bool reverse_dir_  = false;

  // control params
  double wheelbase_=0.33;
  double switch_radius_=1.0;
  double la_base_=0.6, la_gain_=0.5, la_min_=0.7, la_max_=3.0;

  // speed schedule
  double vmin_=1.0, vmax_=5.0, tmin_=0.2, tmax_=4.0;
  double target_v_=1.5;
  rclcpp::Time next_v_change_{0,0,RCL_ROS_TIME};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mt19937 rng_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathDriverNode>());
  rclcpp::shutdown();
  return 0;
}
