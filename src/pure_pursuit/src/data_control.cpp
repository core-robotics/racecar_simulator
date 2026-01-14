// grid_dubins_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
    struct Pose2D
    {
        double x{0}, y{0}, yaw{0};
    };

    double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
    double normAng(double a) { return std::remainder(a, 2.0 * M_PI); }
    double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tfq);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }
    Pose2D vecToPose(const std::vector<double> &v) { return v.size() >= 3 ? Pose2D{v[0], v[1], v[2]} : Pose2D{}; }
    Pose2D toPose2D(const nav_msgs::msg::Odometry &m) { return {m.pose.pose.position.x, m.pose.pose.position.y, yawFromQuat(m.pose.pose.orientation)}; }
    Pose2D toPose2D(const geometry_msgs::msg::PoseStamped &m) { return {m.pose.position.x, m.pose.position.y, yawFromQuat(m.pose.orientation)}; }

    // world point -> base frame of base_w
    Pose2D worldToBase(const Pose2D &base_w, const Pose2D &pt_w)
    {
        double dx = pt_w.x - base_w.x, dy = pt_w.y - base_w.y, c = std::cos(-base_w.yaw), s = std::sin(-base_w.yaw);
        return {c * dx - s * dy, s * dx + c * dy, normAng(pt_w.yaw - base_w.yaw)};
    }
} // namespace

class GridDubinsController : public rclcpp::Node
{
public:
    GridDubinsController() : Node("grid_dubins_controller")
    {
        odom_topic_ = declare_parameter<std::string>("odom_topic", "odom0");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "pose0");
        scan_topic_ = declare_parameter<std::string>("scan_topic", "scan0");
        drive_topic_ = declare_parameter<std::string>("drive_topic", "ackermann_cmd0");

        start_poses_ = {
            vecToPose(declare_parameter<std::vector<double>>("start_pose_1", {-1.5, -0.7, -1.6})),
            vecToPose(declare_parameter<std::vector<double>>("start_pose_2", {1.5, -8.0, 1.6})),
            vecToPose(declare_parameter<std::vector<double>>("start_pose_3", {1.5, -0.7, -1.6})),
            vecToPose(declare_parameter<std::vector<double>>("start_pose_4", {-1.5, -8.0, 1.6}))};

        left_steers_ = declare_parameter<std::vector<double>>("left_steer_grid_rad", {-0.1, -0.2, -0.3, -0.4});
        right_steers_ = declare_parameter<std::vector<double>>("right_steer_grid_rad", {0.1, 0.2, 0.3, 0.4});
        speeds_ = declare_parameter<std::vector<double>>("speed_grid_mps", {2.0, 3.0, 4.0, 5.0});

        wheelbase_m_ = declare_parameter<double>("wheelbase_m", 0.33);
        sim_dt_s_ = declare_parameter<double>("sim_dt_s", 0.05);
        sim_horizon_s_ = declare_parameter<double>("sim_horizon_s", 2.0);
        clear_radius_m_ = declare_parameter<double>("clear_radius_m", 0.60);
        min_valid_range_m_ = declare_parameter<double>("min_valid_range_m", 0.05);

        move_speed_mps_ = declare_parameter<double>("move_speed_mps", 2.0);
        max_steer_rad_ = declare_parameter<double>("max_steer_abs_rad", 0.45);

        goal_tol_m_ = declare_parameter<double>("goal_tol_m", 0.50);
        goal_tol_yaw_rad_ = declare_parameter<double>("goal_tol_yaw_rad", 0.35);

        ttc_done_s_ = declare_parameter<double>("ttc_threshold_s", 1.0);
        front_done_m_ = declare_parameter<double>("front_space_threshold_m", 1.5);

        buildGridPairs();

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            [&](nav_msgs::msg::Odometry::SharedPtr m)
            { odom_pose_=toPose2D(*m); have_odom_=true; });

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, rclcpp::SensorDataQoS(),
            [&](geometry_msgs::msg::PoseStamped::SharedPtr m)
            { pose_pose_=toPose2D(*m); have_pose_=true; });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            [&](sensor_msgs::msg::LaserScan::SharedPtr m)
            { scan_=*m; have_scan_=true; });

        timer_ = create_wall_timer(std::chrono::milliseconds(20), [&]
                                   { tick(); });

        std::cout << "[INIT] starts=" << start_poses_.size() << " grids=" << grid_pairs_.size() << "\n";
    }

private:
    enum class Mode
    {
        WAIT,
        MOVE_TO_START,
        EXECUTE_GRID
    };

    Pose2D curPose() const { return have_pose_ ? pose_pose_ : (have_odom_ ? odom_pose_ : Pose2D{}); }

    void buildGridPairs()
    {
        grid_pairs_.clear();
        for (double s : left_steers_)
            for (double v : speeds_)
                grid_pairs_.push_back({s, v});
        for (double s : right_steers_)
            for (double v : speeds_)
                grid_pairs_.push_back({s, v});
        if (!speeds_.empty())
            grid_pairs_.push_back({0.0, speeds_.front()});
    }

    double rangeAt(double ang) const
    {
        if (!have_scan_)
            return std::numeric_limits<double>::infinity();
        const auto &s = scan_;
        if (ang < s.angle_min || ang > s.angle_max)
            return std::numeric_limits<double>::infinity();
        int i = (int)std::lround((ang - s.angle_min) / s.angle_increment);
        if (i < 0 || i >= (int)s.ranges.size())
            return std::numeric_limits<double>::infinity();
        float r = s.ranges[i];
        if (!std::isfinite(r) || r < min_valid_range_m_)
            return std::numeric_limits<double>::infinity();
        return (double)r;
    }

    // 전방 공간(0rad)만 아주 간단히 체크 (필요하면 ±몇 도 최소값으로 확장 가능)
    double frontSpaceM() const { return rangeAt(0.0); }

    bool hitRay(double ang, double dist) const { return rangeAt(ang) <= dist; }

    std::vector<Pose2D> simLocal(double steer, double speed, double horizon) const
    {
        std::vector<Pose2D> pts;
        Pose2D p{};
        int n = std::max(1, (int)std::ceil(horizon / sim_dt_s_));
        for (int k = 0; k < n; k++)
        {
            pts.push_back(p);
            double yaw_rate = (std::abs(steer) < 1e-6) ? 0.0 : (speed / wheelbase_m_) * std::tan(steer);
            p.x += speed * std::cos(p.yaw) * sim_dt_s_;
            p.y += speed * std::sin(p.yaw) * sim_dt_s_;
            p.yaw = normAng(p.yaw + yaw_rate * sim_dt_s_);
        }
        return pts;
    }

    bool pathFree(const std::vector<Pose2D> &path_local) const
    {
        for (auto &p : path_local)
        {
            double d = std::hypot(p.x, p.y);
            if (d < 1e-6)
                continue;
            double a = std::atan2(p.y, p.x);
            if (hitRay(a, d + clear_radius_m_))
                return false;
        }
        return true;
    }

    double ttcApprox(double steer, double speed) const
    {
        if (!have_scan_ || speed <= 1e-3)
            return std::numeric_limits<double>::infinity();
        auto path = simLocal(steer, speed, sim_horizon_s_);
        double best = std::numeric_limits<double>::infinity();
        for (auto &p : path)
        {
            double d = std::hypot(p.x, p.y);
            if (d < 1e-3)
                continue;
            double a = std::atan2(p.y, p.x);
            if (hitRay(a, d + clear_radius_m_))
                best = std::min(best, d / speed);
        }
        return best;
    }

    bool reached(const Pose2D &cur, const Pose2D &goal) const
    {
        double dx = goal.x - cur.x, dy = goal.y - cur.y;
        return std::hypot(dx, dy) <= goal_tol_m_ && std::abs(normAng(goal.yaw - cur.yaw)) <= goal_tol_yaw_rad_;
    }

    bool chooseDubinsLikeSteer(const Pose2D &cur_w, const Pose2D &goal_w, double &steer_out) const
    {
        Pose2D goal_b = worldToBase(cur_w, goal_w);
        std::vector<double> cand = {-max_steer_rad_, 0.0, max_steer_rad_};
        double best = std::numeric_limits<double>::infinity();
        bool ok = false;

        for (double s : cand)
        {
            auto path = simLocal(s, move_speed_mps_, sim_horizon_s_);
            if (!pathFree(path))
                continue;
            const auto &end = path.back();
            double pos = std::hypot(goal_b.x - end.x, goal_b.y - end.y);
            double yaw = std::abs(normAng(goal_b.yaw - end.yaw));
            double score = pos + 0.5 * yaw;
            if (score < best)
            {
                best = score;
                steer_out = s;
                ok = true;
            }
        }
        return ok;
    }

    std::optional<std::pair<double, double>> pickGrid() const
    {
        for (auto &g : grid_pairs_)
        {
            auto path = simLocal(g.first, g.second, sim_horizon_s_);
            if (pathFree(path))
                return g;
        }
        return std::nullopt;
    }

    void pub(double steer, double speed)
    {
        ackermann_msgs::msg::AckermannDriveStamped m;
        m.header.stamp = now();
        m.drive.steering_angle = clamp(steer, -max_steer_rad_, max_steer_rad_);
        m.drive.speed = std::max(0.0, speed);
        drive_pub_->publish(m);
    }

    void nextStart() { start_idx_ = (start_idx_ + 1) % std::max<size_t>(1, start_poses_.size()); }

    void tick()
    {
        if (!have_scan_ || (!have_odom_ && !have_pose_))
        {
            mode_ = Mode::WAIT;
            pub(0, 0);
            return;
        }

        Pose2D cur = curPose();
        Pose2D goal = start_poses_[start_idx_];

        if (mode_ == Mode::WAIT)
        {
            mode_ = Mode::MOVE_TO_START;
            std::cout << "[MODE] MOVE_TO_START\n";
        }

        if (mode_ == Mode::MOVE_TO_START)
        {
            if (reached(cur, goal))
            {
                mode_ = Mode::EXECUTE_GRID;
                active_grid_.reset();
                std::cout << "[MODE] EXECUTE_GRID start_idx=" << start_idx_ << "\n";
                return;
            }
            double steer = 0.0;
            bool ok = chooseDubinsLikeSteer(cur, goal, steer);
            pub(ok ? steer : 0.0, move_speed_mps_);

            static int k = 0;
            if (++k % 25 == 0)
                std::cout << "[MOVE] start_idx=" << start_idx_ << " steer=" << steer << " cur=(" << cur.x << "," << cur.y << ")\n";
            return;
        }

        if (mode_ == Mode::EXECUTE_GRID)
        {
            if (!active_grid_)
            {
                active_grid_ = pickGrid();
                if (!active_grid_)
                {
                    std::cout << "[GRID] no feasible grid -> switch start\n";
                    nextStart();
                    mode_ = Mode::MOVE_TO_START;
                    pub(0.0, move_speed_mps_);
                    return;
                }
                std::cout << "[GRID] select steer=" << active_grid_->first << " speed=" << active_grid_->second << "\n";
            }

            double steer = active_grid_->first;
            double speed = active_grid_->second;

            // 요구사항: 3번(그리드 실행) 중
            // TTC < 1초 OR 전방 공간 < 1.5m 이면 완료 처리하고 2번 실행 (정지 없음)
            double ttc = ttcApprox(steer, speed);
            double front = frontSpaceM();

            if (ttc < ttc_done_s_ || front < front_done_m_)
            {
                std::cout << "[GRID] done: ttc=" << ttc << " front=" << front << " -> MOVE_TO_START (no stop)\n";
                nextStart();
                mode_ = Mode::MOVE_TO_START;
                active_grid_.reset();
                pub(0.0, move_speed_mps_); // 정지 없이 즉시 다음 단계 command
                return;
            }

            pub(steer, speed);

            static int k = 0;
            if (++k % 25 == 0)
                std::cout << "[GRID] steer=" << steer << " speed=" << speed << " ttc=" << ttc << " front=" << front << "\n";
            return;
        }
    }

private:
    // topics
    std::string odom_topic_, pose_topic_, scan_topic_, drive_topic_;

    // params / config
    std::vector<Pose2D> start_poses_;
    std::vector<double> left_steers_, right_steers_, speeds_;
    std::vector<std::pair<double, double>> grid_pairs_;

    double wheelbase_m_{0.33}, sim_dt_s_{0.05}, sim_horizon_s_{2.0};
    double clear_radius_m_{0.6}, min_valid_range_m_{0.05};
    double move_speed_mps_{2.0}, max_steer_rad_{0.45};
    double goal_tol_m_{0.5}, goal_tol_yaw_rad_{0.35};
    double ttc_done_s_{1.0}, front_done_m_{1.5};

    // ros
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // state
    bool have_odom_{false}, have_pose_{false}, have_scan_{false};
    Pose2D odom_pose_, pose_pose_;
    sensor_msgs::msg::LaserScan scan_;
    Mode mode_{Mode::WAIT};
    size_t start_idx_{0};
    std::optional<std::pair<double, double>> active_grid_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridDubinsController>());
    rclcpp::shutdown();
    return 0;
}
