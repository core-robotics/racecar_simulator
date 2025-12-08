// ROS2 lap stats node: 3min eval, index-based SF crossing, lap times & collisions

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <chrono>
#include <cmath>
#include <sstream>
#include <limits>
#include <vector>

class RaceStatsNode : public rclcpp::Node
{
public:
  RaceStatsNode() : Node("race_stats_node")
  {
    odom_topic_           = declare_parameter<std::string>("odom_topic", "odom0");
    collision_topic_      = declare_parameter<std::string>("collision_topic", "collision0");
    path_topic_           = declare_parameter<std::string>("path_topic", "center_path");
    cmd_topic_            = declare_parameter<std::string>("cmd_topic", "ackermann_cmd0");
    text_frame_           = declare_parameter<std::string>("text_frame", "map");
    fixed_frame_          = declare_parameter<std::string>("fixed_frame", "map");
    text_anchor_x_        = declare_parameter<double>("text_anchor_x", 0.0);
    text_anchor_y_        = declare_parameter<double>("text_anchor_y", 0.0);
    text_scale_           = declare_parameter<double>("text_scale", 0.25);
    min_lap_time_         = declare_parameter<double>("min_lap_time", 3.0);
    update_rate_hz_       = declare_parameter<double>("update_rate_hz", 10.0);
    start_on_first_cross_ = declare_parameter<bool>("start_on_first_cross", true);
    eval_duration_sec_    = declare_parameter<double>("eval_duration_sec", 180.0);
    sf_radius_            = declare_parameter<double>("sf_radius", 1.0);

    auto b_qos   = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto r_t_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, b_qos, std::bind(&RaceStatsNode::onOdom, this, std::placeholders::_1));
    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
        collision_topic_, b_qos, std::bind(&RaceStatsNode::onCollision, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, r_t_qos, std::bind(&RaceStatsNode::onCenterPath, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        cmd_topic_, b_qos, std::bind(&RaceStatsNode::onCmd, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/race_stats_markers", r_t_qos);

    auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(period, std::bind(&RaceStatsNode::onTimer, this));
  }

private:
  // parameters / topics
  std::string odom_topic_, collision_topic_, path_topic_, cmd_topic_, text_frame_, fixed_frame_;
  double text_anchor_x_{0.0}, text_anchor_y_{0.0}, text_scale_{0.25};
  double min_lap_time_{3.0};
  double update_rate_hz_{10.0};
  bool   start_on_first_cross_{true};
  double eval_duration_sec_{180.0};
  double eval_elapsed_sec_{0.0};
  double sf_radius_{1.0};

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     collision_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // center path
  std::vector<double> px_;
  std::vector<double> py_;
  int  path_size_{0};
  bool have_path_{false};

  // SF line
  bool   line_published_{false};
  double p0x_{0.0}, p0y_{0.0};
  double tx_{0.0}, ty_{1.0};
  double n0x_{1.0}, n0y_{0.0};

  // nearest index tracking
  bool have_prev_idx_{false};
  int  prev_idx_{0};

  // stats
  int    lap_count_{0};
  int    collision_count_{0};
  int    current_lap_collisions_{0};
  bool   prev_collision_{false};

  rclcpp::Time eval_start_time_{};
  rclcpp::Time lap_start_time_{};
  rclcpp::Time last_cross_time_{};

  bool   eval_running_{false};
  bool   eval_finished_{false};
  bool   running_lap_{false};

  double last_lap_time_sec_{0.0};
  double best_lap_time_sec_{std::numeric_limits<double>::infinity()};
  double current_lap_time_sec_{0.0};

  std::vector<double> lap_times_;
  std::vector<int>    lap_collisions_;

  std::string last_text_;

  static double norm(double x, double y) { return std::sqrt(x * x + y * y); }

  void resetStatsForEval()
  {
    lap_count_              = 0;
    collision_count_        = 0;
    current_lap_collisions_ = 0;
    prev_collision_         = false;
    lap_start_time_         = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_cross_time_        = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_lap_time_sec_      = 0.0;
    best_lap_time_sec_      = std::numeric_limits<double>::infinity();
    current_lap_time_sec_   = 0.0;
    lap_times_.clear();
    lap_collisions_.clear();
    have_prev_idx_          = false;
  }

  void onCenterPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    const auto &poses = msg->poses;
    if (poses.size() < 2)
      return;

    px_.clear();
    py_.clear();
    px_.reserve(poses.size());
    py_.reserve(poses.size());
    for (const auto &ps : poses)
    {
      px_.push_back(ps.pose.position.x);
      py_.push_back(ps.pose.position.y);
    }
    path_size_ = static_cast<int>(px_.size());

    p0x_ = px_[0];
    p0y_ = py_[0];
    double x1 = px_[1];
    double y1 = py_[1];

    double dx = x1 - p0x_;
    double dy = y1 - p0y_;
    double L  = norm(dx, dy);
    if (L < 1e-6) L = 1e-6;

    n0x_ = dx / L;
    n0y_ = dy / L;
    tx_  = -n0y_;
    ty_  =  n0x_;

    have_path_      = true;
    line_published_ = true;
    
  }

  void onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr /*msg*/)
  {
    if (!eval_running_ && !eval_finished_)
    {
      eval_start_time_  = this->get_clock()->now();
      eval_running_     = true;
      eval_elapsed_sec_ = 0.0;
      resetStatsForEval();
      RCLCPP_INFO(this->get_logger(), "Evaluation window started (%.1f s)", eval_duration_sec_);
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto now = this->get_clock()->now();

    if (eval_running_)
    {
      eval_elapsed_sec_ = (now - eval_start_time_).seconds();
      if (eval_elapsed_sec_ >= eval_duration_sec_)
      {
        eval_running_     = false;
        eval_finished_    = true;
        eval_elapsed_sec_ = eval_duration_sec_;
        RCLCPP_INFO(this->get_logger(), "Evaluation window finished");
      }
    }

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    if (eval_running_ && running_lap_)
      current_lap_time_sec_ = (now - lap_start_time_).seconds();

    if (!have_path_ || path_size_ == 0)
      return;

    int    idx    = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < path_size_; ++i)
    {
      double dx = x - px_[i];
      double dy = y - py_[i];
      double d2 = dx * dx + dy * dy;
      if (d2 < best_d2)
      {
        best_d2 = d2;
        idx     = i;
      }
    }
    // std::cout<< "prev_idx_: " << prev_idx_ << " idx: " << idx << " path size: " << path_size_ << std::endl;

    if (!have_prev_idx_)
    {
      prev_idx_     = idx;
      have_prev_idx_= true;
      // std::cout<< "No previous index" << std::endl;
    }

    bool crossed = false;

    // std::cout<< "Prev idx: " << prev_idx_ << " Curr idx: " << idx << std::endl;

    if (path_size_ >= 6)
    {
      int last_zone_start = path_size_ - 2;
      bool was_in_last  = (prev_idx_ >= last_zone_start);
      bool now_in_first = (idx <= 1);
      if (was_in_last && now_in_first)
        crossed = true;

      // std::cout<< "last_zone_start: " << last_zone_start
      //          << " was_in_last: " << was_in_last
      //          << " now_in_first: " << now_in_first
      //          << " crossed: " << crossed
      //          << " path size: " << path_size_ << std::endl;
    }
    else
    {
      bool was_in_last  = (prev_idx_ >= path_size_ - 1);
      bool now_in_first = (idx == 0);
      if (was_in_last && now_in_first)
        crossed = true;
    }

    if (crossed && eval_running_)
    {
      const rclcpp::Time now_t = now;
      const double since_lap_start =
          lap_start_time_.nanoseconds() > 0
              ? (now_t - lap_start_time_).seconds()
              : std::numeric_limits<double>::infinity();

      bool accepted = false;

      if (running_lap_)
      {
        if (since_lap_start > min_lap_time_)
        {
          const double lap_time = (now_t - lap_start_time_).seconds();
          last_lap_time_sec_ = lap_time;
          lap_times_.push_back(lap_time);
          lap_collisions_.push_back(current_lap_collisions_);
          if (lap_time < best_lap_time_sec_)
            best_lap_time_sec_ = lap_time;
          lap_count_++;
          lap_start_time_         = now_t;
          current_lap_collisions_ = 0;
          accepted = true;
        }
      }
      else if (start_on_first_cross_)
      {
        running_lap_            = true;
        lap_start_time_         = now_t;
        current_lap_collisions_ = 0;
        accepted                = true;
      }

      if (accepted)
        last_cross_time_ = now_t;
    }

    prev_idx_ = idx;
  }

  void onCollision(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool coll = msg->data;
    if (coll && !prev_collision_)
    {
      collision_count_++;
      if (running_lap_)
        current_lap_collisions_++;
    }
    prev_collision_ = coll;
  }

  void onTimer()
  {
    if (eval_running_)
    {
      eval_elapsed_sec_ = (this->get_clock()->now() - eval_start_time_).seconds();
      if (eval_elapsed_sec_ >= eval_duration_sec_)
      {
        eval_running_     = false;
        eval_finished_    = true;
        eval_elapsed_sec_ = eval_duration_sec_;
      }
    }
    publishMarkers();
  }

  void publishMarkers()
  {
    builtin_interfaces::msg::Time stamp{};
    visualization_msgs::msg::MarkerArray arr;

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << (eval_finished_ ? "(FINISHED)\n" : "(RUNNING)\n")
        << "Eval: " << eval_elapsed_sec_ << " / " << eval_duration_sec_ << " s\n"
        << "Current lap: " << ((eval_running_ && running_lap_) ? current_lap_time_sec_ : 0.0) << " s\n"
        << "Current collisions: " << ((eval_running_ && running_lap_) ? current_lap_collisions_ : 0) << "\n";

    if (!lap_times_.empty())
    {
      oss << "\nLap history (" << lap_times_.size() << "):\n";
      for (size_t i = 0; i < lap_times_.size(); ++i)
      {
        int c = (i < lap_collisions_.size()) ? lap_collisions_[i] : 0;
        oss << " #" << (i + 1) << ": " << lap_times_[i]
            << " Collisions: " << c << "\n";
      }
    }

    std::string new_text = oss.str();
    if (new_text == last_text_ && (have_path_ ? line_published_ : true))
      return;

    visualization_msgs::msg::Marker text;
    text.header.frame_id      = text_frame_;
    text.header.stamp         = stamp;
    text.ns                   = "race_stats";
    text.id                   = 1;
    text.type                 = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action               = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x      = text_anchor_x_;
    text.pose.position.y      = text_anchor_y_;
    text.pose.position.z      = 0.5;
    text.pose.orientation.w   = 1.0;
    text.scale.z              = text_scale_;
    text.color.a              = 1.0;
    text.color.r              = 1.0;
    text.color.g              = 1.0;
    text.color.b              = 1.0;
    text.lifetime             = rclcpp::Duration(0, 0);
    text.text                 = new_text;
    arr.markers.push_back(text);
    last_text_ = new_text;

    if (have_path_)
    {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = fixed_frame_;
      line.header.stamp    = stamp;
      line.ns              = "race_stats";
      line.id              = 2;
      line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action          = visualization_msgs::msg::Marker::ADD;
      line.scale.x         = 0.03;
      line.color.a         = 1.0;
      line.color.r         = 1.0;
      line.color.g         = 0.2;
      line.color.b         = 0.2;
      line.lifetime        = rclcpp::Duration(0, 0);

      geometry_msgs::msg::Point pA, pB;
      const double half_len = 1.6;
      pA.x = p0x_ - half_len * tx_;
      pA.y = p0y_ - half_len * ty_;
      pA.z = 0.05;
      pB.x = p0x_ + half_len * tx_;
      pB.y = p0y_ + half_len * ty_;
      pB.z = 0.05;
      line.points.push_back(pA);
      line.points.push_back(pB);

      arr.markers.push_back(line);
      line_published_ = true;
    }

    marker_pub_->publish(arr);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaceStatsNode>());
  rclcpp::shutdown();
  return 0;
}
