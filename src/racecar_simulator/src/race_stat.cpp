// ROS2 lap stats node: 3min eval, SF crossing, lap times & lap collisions

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
    // Parameters
    odom_topic_          = declare_parameter<std::string>("odom_topic", "odom0");
    collision_topic_     = declare_parameter<std::string>("collision_topic", "collision0");
    path_topic_          = declare_parameter<std::string>("path_topic", "center_path");
    cmd_topic_           = declare_parameter<std::string>("cmd_topic", "ackermann_cmd0");
    text_frame_          = declare_parameter<std::string>("text_frame", "map");
    fixed_frame_         = declare_parameter<std::string>("fixed_frame", "map");
    text_anchor_x_       = declare_parameter<double>("text_anchor_x", 0.0);
    text_anchor_y_       = declare_parameter<double>("text_anchor_y", 0.0);
    text_scale_          = declare_parameter<double>("text_scale", 0.25);
    min_lap_time_        = declare_parameter<double>("min_lap_time", 3.0);
    update_rate_hz_      = declare_parameter<double>("update_rate_hz", 10.0);
    start_on_first_cross_= declare_parameter<bool>("start_on_first_cross", true);
    eval_duration_sec_   = declare_parameter<double>("eval_duration_sec", 180.0);
    sf_radius_           = declare_parameter<double>("sf_radius", 1.0);

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
  // params / topics
  std::string odom_topic_, collision_topic_, path_topic_, cmd_topic_, text_frame_, fixed_frame_;
  double text_anchor_x_{0.0}, text_anchor_y_{0.0}, text_scale_{0.25};
  double min_lap_time_{3.0};
  double update_rate_hz_{10.0};
  bool   start_on_first_cross_{true};
  double eval_duration_sec_{180.0};
  double eval_elapsed_sec_{0.0};
  double sf_radius_{1.0};

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // SF line (from path)
  bool   have_path_{false};
  bool   line_published_{false};
  double p0x_{0.0}, p0y_{0.0};  // SF point
  double tx_{0.0}, ty_{1.0};    // SF line direction
  double n0x_{1.0}, n0y_{0.0};  // normal (path tangent)
  bool   have_prev_sd_{false};
  double prev_sd_{0.0};

  // stats
  int    lap_count_{0};
  int    collision_count_{0};
  int    current_lap_collisions_{0};
  bool   prev_collision_{false};

  rclcpp::Time eval_start_time_{};
  rclcpp::Time lap_start_time_{};
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
    lap_count_             = 0;
    collision_count_       = 0;
    current_lap_collisions_= 0;
    prev_collision_        = false;
    lap_start_time_        = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_lap_time_sec_     = 0.0;
    best_lap_time_sec_     = std::numeric_limits<double>::infinity();
    current_lap_time_sec_  = 0.0;
    lap_times_.clear();
    lap_collisions_.clear();
    have_prev_sd_          = false;
  }

  // set SF line from center_path (first segment)
  void onCenterPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    const auto &poses = msg->poses;
    if (poses.size() < 2)
      return;

    p0x_ = poses[0].pose.position.x;
    p0y_ = poses[0].pose.position.y;
    double x1 = poses[1].pose.position.x;
    double y1 = poses[1].pose.position.y;

    double dx = x1 - p0x_;
    double dy = y1 - p0y_;
    double L  = norm(dx, dy);
    if (L < 1e-6) L = 1e-6;

    n0x_ = dx / L;
    n0y_ = dy / L;
    tx_  = -n0y_;
    ty_  =  n0x_;

    have_path_      = true;
    line_published_ = false;
    have_prev_sd_   = false;
  }

  // signed distance to SF line
  inline double signedDistSF(double x, double y) const
  {
    return (x - p0x_) * n0x_ + (y - p0y_) * n0y_;
  }

  // start eval on first cmd
  void onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr /*msg*/)
  {
    if (!eval_running_ && !eval_finished_)
    {
      eval_start_time_   = this->get_clock()->now();
      eval_running_      = true;
      eval_elapsed_sec_  = 0.0;
      resetStatsForEval();
      RCLCPP_INFO(this->get_logger(), "Evaluation window started (%.1f s)", eval_duration_sec_);
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto now = this->get_clock()->now();

    // eval timer
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

    if (!have_path_)
      return;

    const double sd = signedDistSF(x, y);

    if (!have_prev_sd_)
    {
      prev_sd_     = sd;
      have_prev_sd_= true;
      return;
    }

    // SF crossing: sign flip near SF point
    bool crossed = false;
    const bool sign_flip = (prev_sd_ <= 0.0 && sd > 0.0) ||
                           (prev_sd_ >= 0.0 && sd < 0.0);
    if (sign_flip)
    {
      const double d_p0 = std::hypot(x - p0x_, y - p0y_);
      if (d_p0 < sf_radius_)
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
          lap_start_time_        = now_t;
          current_lap_collisions_= 0;
          accepted = true;
        }
      }
      else if (start_on_first_cross_)
      {
        running_lap_           = true;
        lap_start_time_        = now_t;
        current_lap_collisions_= 0;
        accepted               = true;
      }

      if (accepted)
        last_cross_time_ = now_t;
    }

    prev_sd_ = sd;
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
    oss << "Eval: " << eval_elapsed_sec_ << " / " << eval_duration_sec_ << " s\n"
        << (eval_finished_ ? " (FINISHED)\n" : "\n")
        // << "Lap count: " << lap_count_ << "\n"
        << "Current lap: " << ((eval_running_ && running_lap_) ? current_lap_time_sec_ : 0.0) << " s\n"
        << "Current collisions: " << ((eval_running_ && running_lap_) ? current_lap_collisions_ : 0) << "\n";

    if (!lap_times_.empty())
    {
      oss << "\nLap history (" << lap_times_.size() << "):\n";
      for (size_t i = 0; i < lap_times_.size(); ++i)
      {
        int c = (i < lap_collisions_.size()) ? lap_collisions_[i] : 0;
        oss << "  #" << (i + 1) << ": " << lap_times_[i]
            << "  Collisions: " << c << "\n";
      }
    }

    std::string new_text = oss.str();
    if (new_text == last_text_ && (have_path_ ? line_published_ : true))
      return;

    visualization_msgs::msg::Marker text;
    text.header.frame_id = text_frame_;
    text.header.stamp    = stamp;
    text.ns              = "race_stats";
    text.id              = 1;
    text.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action          = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = text_anchor_x_;
    text.pose.position.y = text_anchor_y_;
    text.pose.position.z = 0.5;
    text.pose.orientation.w = 1.0;
    text.scale.z         = text_scale_;
    text.color.a         = 1.0;
    text.color.r         = 1.0;
    text.color.g         = 1.0;
    text.color.b         = 1.0;
    text.lifetime        = rclcpp::Duration(0, 0);
    text.text            = new_text;
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
      const double half_len = 1.0;
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

  rclcpp::Time last_cross_time_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaceStatsNode>());
  rclcpp::shutdown();
  return 0;
}
