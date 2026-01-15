#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
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

struct Point2D
{
    double x = 0.0, y = 0.0;
};

static inline double wrapAngle(double a)
{
    while (a > M_PI)
        a -= 2 * M_PI;
    while (a < -M_PI)
        a += 2 * M_PI;
    return a;
}
static inline double sqr(double v) { return v * v; }
static inline double dist2(const Point2D &a, const Point2D &b) { return sqr(a.x - b.x) + sqr(a.y - b.y); }

static inline double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tq);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

static inline Point2D projectToSegment(const Point2D &p, const Point2D &a, const Point2D &b, double &t01)
{
    const double vx = b.x - a.x, vy = b.y - a.y;
    const double wx = p.x - a.x, wy = p.y - a.y;
    const double denom = vx * vx + vy * vy;
    double t = (denom > 1e-12) ? (wx * vx + wy * vy) / denom : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    t01 = t;
    return {a.x + t * vx, a.y + t * vy};
}

class PathDriverNode final : public rclcpp::Node
{
public:
    PathDriverNode() : rclcpp::Node("path_driver_node")
    {
        pose_topic_ = declare_parameter<std::string>("pose_topic", "pose0");
        drive_topic_ = declare_parameter<std::string>("drive_topic", "ackermann_cmd0");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "odom0");
        path_topic_ = declare_parameter<std::string>("path_topic", "active_path");
        frame_id_ = declare_parameter<std::string>("frame_id", "map");

        // geometry / switching
        margin_ = declare_parameter<double>("margin", 0.20);
        samples_ = declare_parameter<int>("samples", 160);
        wheelbase_m_ = declare_parameter<double>("wheelbase", 0.33);
        switch_radius_ = declare_parameter<double>("switch_radius", 1.50);

        // lookahead (speed-based)
        lookahead_base_ = declare_parameter<double>("lookahead_base", 0.7);
        lookahead_gain_ = declare_parameter<double>("lookahead_gain", 0.4);
        lookahead_min_ = declare_parameter<double>("lookahead_min", 0.7);
        lookahead_max_ = declare_parameter<double>("lookahead_max", 3.0);

        // speed randomization
        speed_min_ = declare_parameter<double>("speed_min", 1.0);
        speed_max_ = declare_parameter<double>("speed_max", 5.0);
        change_min_s_ = declare_parameter<double>("change_min_s", 0.2);
        change_max_s_ = declare_parameter<double>("change_max_s", 3.0);
        if (speed_min_ > speed_max_)
            std::swap(speed_min_, speed_max_);
        if (change_min_s_ > change_max_s_)
            std::swap(change_min_s_, change_max_s_);
        change_min_s_ = std::max(1e-3, change_min_s_);
        change_max_s_ = std::max(change_min_s_, change_max_s_);

        // track bounds (as in your code)
        x_min_ = -2.0;
        x_max_ = 2.0;
        y_min_ = -9.0;
        y_max_ = -0.5;

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
            { last_pose_ = *msg; });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg)
            { last_odom_ = *msg; });

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 1);

        rng_.seed(std::random_device{}());
        buildTemplates();

        reverse_path_ = coinFlip();
        path_start_ = Endpoint::Top;
        selectNextPath(0.0);

        rescheduleSpeed();
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]
                                   { onTick(); });
    }

private:
    enum class Endpoint
    {
        Top,
        Bottom
    };
    enum class TemplateId
    {
        CapsuleR,
        CapsuleL,
        EightS1,
        EightS2
    };

    Endpoint pathEnd() const { return (path_start_ == Endpoint::Top) ? Endpoint::Bottom : Endpoint::Top; }
    const Point2D &endpointPoint(Endpoint e) const { return (e == Endpoint::Top) ? top_ : bottom_; }

    double randUniform(double a, double b)
    {
        std::uniform_real_distribution<double> d(a, b);
        return d(rng_);
    }
    bool coinFlip()
    {
        std::uniform_int_distribution<int> d(0, 1);
        return d(rng_) == 1;
    }

    void rescheduleSpeed()
    {
        target_speed_ = randUniform(speed_min_, speed_max_);
        const double dt = randUniform(change_min_s_, change_max_s_);
        next_speed_change_ = now() + rclcpp::Duration::from_seconds(dt);
    }

    double lookaheadForSpeed(double v) const
    {
        const double L = lookahead_base_ + lookahead_gain_ * v;
        return std::clamp(L, lookahead_min_, lookahead_max_);
    }

    struct ClosestArc
    {
        double s = 0.0;  // arc-length position on path
        int seg = 0;     // segment index i (between i and i+1)
        double t = 0.0;  // interpolation on that segment
        Point2D point{}; // closest point on the polyline
    };

    ClosestArc closestArcOnPath(const Point2D &pos) const
    {
        ClosestArc best;
        double best_d2 = 1e300;

        const int n = (int)active_path_.size();
        if (n < 2)
            return best;

        for (int i = 0; i < n - 1; ++i)
        {
            double t01 = 0.0;
            const Point2D q = projectToSegment(pos, active_path_[i], active_path_[i + 1], t01);
            const double d2 = dist2(pos, q);
            if (d2 < best_d2)
            {
                best_d2 = d2;
                best.seg = i;
                best.t = t01;
                best.point = q;
                best.s = arc_prefix_[i] + t01 * (arc_prefix_[i + 1] - arc_prefix_[i]);
            }
        }
        return best;
    }

    Point2D pointAtArc(double s) const
    {
        const int n = (int)active_path_.size();
        if (n == 0)
            return {};
        if (n == 1)
            return active_path_[0];

        const double s_end = arc_prefix_.back();
        s = std::clamp(s, 0.0, s_end);

        // find i such that arc_prefix_[i] <= s <= arc_prefix_[i+1]
        int i = (int)(std::upper_bound(arc_prefix_.begin(), arc_prefix_.end(), s) - arc_prefix_.begin()) - 1;
        i = std::clamp(i, 0, n - 2);

        const double s0 = arc_prefix_[i];
        const double s1 = arc_prefix_[i + 1];
        const double t = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;

        const auto &a = active_path_[i];
        const auto &b = active_path_[i + 1];
        return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
    }

    void onTick()
    {
        if (!last_pose_ || active_path_.size() < 2)
            return;

        if (now() >= next_speed_change_)
            rescheduleSpeed();

        const auto &pose = last_pose_->pose;
        const Point2D pos{pose.position.x, pose.position.y};
        const double yaw = yawFromQuat(pose.orientation);

        const Point2D &end_pt = endpointPoint(pathEnd());
        if (dist2(pos, end_pt) < sqr(switch_radius_))
        {
            path_start_ = pathEnd();
            selectNextPath(yaw);
            return;
        }

        // (1) segment-projection closest + arc-length lookahead
        const ClosestArc closest = closestArcOnPath(pos);

        // (2) speed-based lookahead
        const double L_cmd = lookaheadForSpeed(last_odom_ ? last_odom_->twist.twist.linear.x : 0.0);
        const Point2D tgt = pointAtArc(closest.s + L_cmd);

        // compute curvature using target point in vehicle frame (stable)
        const double dx = tgt.x - pos.x;
        const double dy = tgt.y - pos.y;
        const double c = std::cos(yaw), s = std::sin(yaw);
        const double x_r = c * dx + s * dy;
        const double y_r = -s * dx + c * dy;
        const double L = std::max(1e-3, std::hypot(x_r, y_r));
        const double kappa = (2.0 * y_r) / (L * L);
        const double steer = std::atan(wheelbase_m_ * kappa);

        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = now();
        cmd.header.frame_id = frame_id_;
        cmd.drive.steering_angle = steer;
        cmd.drive.speed = target_speed_;
        std::cout << "target speed: " << target_speed_ << " m/s\n"
                    << "speed: " << (last_odom_ ? last_odom_->twist.twist.linear.x : 0.0) << " m/s\n"
                    << "steer: " << steer << " rad\n\n";
        drive_pub_->publish(cmd);
    }

    void selectNextPath(double yaw)
    {
        const TemplateId capsule = bestForwardTemplate({TemplateId::CapsuleR, TemplateId::CapsuleL}, yaw);
        const TemplateId eight = bestForwardTemplate({TemplateId::EightS1, TemplateId::EightS2}, yaw);
        const TemplateId chosen = coinFlip() ? capsule : eight;

        active_path_ = orientPath(pathFromTemplate(chosen));
        buildArcPrefix();
        publishActivePath();
    }

    TemplateId bestForwardTemplate(const std::initializer_list<TemplateId> &cands, double yaw)
    {
        const double fx = std::cos(yaw), fy = std::sin(yaw);
        TemplateId best = *cands.begin();
        double best_dot = -1e300;

        for (auto t : cands)
        {
            auto p = orientPath(pathFromTemplate(t));
            if (p.size() < 2)
                continue;
            const double vx = p[1].x - p[0].x, vy = p[1].y - p[0].y;
            const double dot = fx * vx + fy * vy;
            if (dot > 0.0)
                return t;
            if (dot > best_dot)
            {
                best_dot = dot;
                best = t;
            }
        }
        return best;
    }

    std::vector<Point2D> orientPath(std::vector<Point2D> path) const
    {
        if (path_start_ == Endpoint::Bottom)
            std::reverse(path.begin(), path.end());
        if (reverse_path_)
            std::reverse(path.begin(), path.end());
        return path;
    }

    void buildArcPrefix()
    {
        const int n = (int)active_path_.size();
        arc_prefix_.assign(n, 0.0);
        for (int i = 1; i < n; ++i)
        {
            const auto &a = active_path_[i - 1];
            const auto &b = active_path_[i];
            arc_prefix_[i] = arc_prefix_[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
        }
    }

    void buildTemplates()
    {
        const double w = x_max_ - x_min_, h = y_max_ - y_min_;
        const double r_w = 0.5 * w - margin_;
        const double r_h = (h - 2.0 * margin_) / 4.0;
        radius_ = std::max(0.05, std::min(r_w, r_h));

        const double cy = 0.5 * (y_min_ + y_max_);
        top_ = {0.0, cy - 2.0 * radius_};
        bottom_ = {0.0, cy + 2.0 * radius_};

        capsule_r_ = capsuleSide(true, cy);
        capsule_l_ = capsuleSide(false, cy);
        eight_s1_ = eightS(true, false, cy);
        eight_s2_ = eightS(false, true, cy);

        clampToBounds(capsule_r_);
        clampToBounds(capsule_l_);
        clampToBounds(eight_s1_);
        clampToBounds(eight_s2_);
    }

    std::vector<Point2D> capsuleSide(bool right, double cy)
    {
        const double x = right ? +radius_ : -radius_;
        const double yt = cy - radius_, yb = cy + radius_;
        const int n_arc = std::max(2, samples_ / 4);
        const int n_side = std::max(2, samples_ / 2);

        auto top_arc = arcPoints(0.0, yt, -M_PI / 2.0, right ? 0.0 : -M_PI, n_arc);

        std::vector<Point2D> side;
        side.reserve(n_side);
        for (int i = 0; i < n_side; i++)
        {
            const double u = double(i) / double(n_side - 1);
            side.push_back({x, yt + (yb - yt) * u});
        }

        auto bot_arc = right
                           ? arcPoints(0.0, yb, 0.0, M_PI / 2.0, n_arc)
                           : arcPoints(0.0, yb, -M_PI, -3.0 * M_PI / 2.0, n_arc);

        std::vector<Point2D> out = top_arc;
        out.insert(out.end(), side.begin() + 1, side.end());
        out.insert(out.end(), bot_arc.begin() + 1, bot_arc.end());
        return out;
    }

    std::vector<Point2D> eightS(bool top_right, bool bottom_right, double cy)
    {
        const double yt = cy - radius_, yb = cy + radius_;
        const int n = std::max(2, samples_ / 2);

        auto top_arc = arcPoints(0.0, yt, -M_PI / 2.0, top_right ? +M_PI / 2.0 : -3.0 * M_PI / 2.0, n);
        auto bot_arc = arcPoints(0.0, yb, -M_PI / 2.0, bottom_right ? +M_PI / 2.0 : -3.0 * M_PI / 2.0, n);

        top_arc.insert(top_arc.end(), bot_arc.begin() + 1, bot_arc.end());
        return top_arc;
    }

    std::vector<Point2D> arcPoints(double cx, double cy, double a0, double a1, int n) const
    {
        n = std::max(2, n);
        std::vector<Point2D> pts;
        pts.reserve(n);
        for (int i = 0; i < n; i++)
        {
            const double u = double(i) / double(n - 1);
            const double a = a0 + (a1 - a0) * u;
            pts.push_back({cx + radius_ * std::cos(a), cy + radius_ * std::sin(a)});
        }
        return pts;
    }

    void clampToBounds(std::vector<Point2D> &pts) const
    {
        const double xmin = x_min_ + margin_, xmax = x_max_ - margin_;
        const double ymin = y_min_ + margin_, ymax = y_max_ - margin_;
        for (auto &p : pts)
        {
            p.x = std::min(std::max(p.x, xmin), xmax);
            p.y = std::min(std::max(p.y, ymin), ymax);
        }
    }

    std::vector<Point2D> pathFromTemplate(TemplateId t) const
    {
        switch (t)
        {
        case TemplateId::CapsuleR:
            return capsule_r_;
        case TemplateId::CapsuleL:
            return capsule_l_;
        case TemplateId::EightS1:
            return eight_s1_;
        case TemplateId::EightS2:
            return eight_s2_;
        }
        return capsule_r_;
    }

    void publishActivePath()
    {
        nav_msgs::msg::Path msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.poses.reserve(active_path_.size());

        for (const auto &p : active_path_)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        path_pub_->publish(msg);
    }

private:
    std::string pose_topic_, drive_topic_, path_topic_, odom_topic_, frame_id_;

    // bounds / template
    double x_min_ = -2.0, x_max_ = 2.0, y_min_ = -9.0, y_max_ = -0.5;
    double margin_ = 0.2;
    int samples_ = 160;
    double radius_ = 1.0;
    Point2D top_{}, bottom_{};
    std::vector<Point2D> capsule_r_, capsule_l_, eight_s1_, eight_s2_;

    // active path + arc-length prefix
    std::vector<Point2D> active_path_;
    std::vector<double> arc_prefix_;

    // state
    std::optional<geometry_msgs::msg::PoseStamped> last_pose_;
    std::optional<nav_msgs::msg::Odometry> last_odom_;
    Endpoint path_start_ = Endpoint::Top;
    bool reverse_path_ = false;

    // control
    double wheelbase_m_ = 0.33;
    double switch_radius_ = 1.50;

    // speed-based lookahead params
    double lookahead_base_ = 0.7, lookahead_gain_ = 0.4, lookahead_min_ = 0.7, lookahead_max_ = 3.0;

    // speed randomization
    double speed_min_ = 1.0, speed_max_ = 5.0;
    double change_min_s_ = 0.2, change_max_s_ = 3.0;
    double target_speed_ = 1.5;
    rclcpp::Time next_speed_change_{0, 0, RCL_ROS_TIME};

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mt19937 rng_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathDriverNode>());
    rclcpp::shutdown();
    return 0;
}
