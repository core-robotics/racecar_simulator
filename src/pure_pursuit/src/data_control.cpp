#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

class DataControl : public rclcpp::Node
{
public:
    DataControl() : Node("data_control_node")
    {
        // Speeds
        explore_speed_mps_ = declare_parameter<double>("explore_speed_mps", 1.0);
        explore_risk_speed_mps_ = declare_parameter<double>("explore_risk_speed_mps", 1.0);

        // Grid timing
        grid_hold_sec_ = declare_parameter<double>("grid_hold_time_sec", 1.0);
        grid_cooldown_sec_ = declare_parameter<double>("grid_trigger_cooldown_sec", 3.0);

        // Explore steering
        explore_fov_half_deg_ = declare_parameter<double>("explore_fov_half_deg", 90.0);
        explore_window_half_deg_ = declare_parameter<double>("explore_window_half_deg", 7.0);
        steer_limit_rad_ = declare_parameter<double>("steering_limit_rad", 0.42);

        // Grid safety fan
        grid_horizon_mul_ = declare_parameter<double>("grid_distance_horizon_multiplier", 2.0);
        grid_margin_m_ = declare_parameter<double>("grid_distance_margin_m", 1.0);
        fan_base_half_deg_ = declare_parameter<double>("grid_fan_base_half_deg", 10.0);
        fan_half_gain_deg_per_rad_ = declare_parameter<double>("grid_fan_half_gain_deg_per_rad", 25.0);
        fan_center_gain_deg_per_rad_ = declare_parameter<double>("grid_fan_center_gain_deg_per_rad", 30.0);
        require_both_clear_ = declare_parameter<bool>("require_both_points_clear", true);

        // Scan freshness
        scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.2);

        // TTC risk
        ttc_enter_sec_ = declare_parameter<double>("ttc_risk_enter_sec", 0.7);
        ttc_exit_sec_ = declare_parameter<double>("ttc_risk_exit_sec", 1.2);
        ttc_exit_hold_sec_ = declare_parameter<double>("ttc_risk_exit_hold_sec", 0.5);

        // Start Pose
        left_start_pose_1_ = declare_parameter<double>("left_start_pose_1", -2.0,-0.2, -1.6);
        left_start_pose_2_ = declare_parameter<double>("left_start_pose_2", 2.0, -8.5, 1.6);
        right_start_pose_1_ = declare_parameter<double>("right_start_pose_1", 2.0, -0.2, -1.6);
        right_start_pose_2_ = declare_parameter<double>("right_start_pose_2", -2.0, -8.5, 1.6);

        // Grids
        left_steer_grid_rad_ = declare_parameter<std::vector<double>>(
            "left_steer_grid_rad", std::vector<double>{0.1, 0.2, 0.3, 0.4});
        right_steer_grid_rad_ = declare_parameter<std::vector<double>>(
            "right_steer_grid_rad", std::vector<double>{-0.1, -0.2, -0.3, -0.4});
        speed_grid_mps_ = declare_parameter<std::vector<double>>(
            "speed_grid_mps", std::vector<double>{2.0, 3.0, 4.0, 5.0});

        // Topics
        odom_topic_ = declare_parameter<std::string>("odom_topic", "odom0");
        scan_topic_ = declare_parameter<std::string>("scan_topic", "scan0");
        drive_topic_ = declare_parameter<std::string>("drive_topic", "ackermann_cmd0");

        buildGridPoints();
        buildGridPairs();
        pair_done_.assign(grid_pairs_.size(), 0);

        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, pub_qos);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, sub_qos,
            [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                odom_ = *msg;
                has_odom_ = true;
            });
        
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose0", sub_qos,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                pose_ = *msg;
                has_pose_ = true;
            });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, sub_qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                scan_ = *msg;
                has_scan_ = true;
                last_scan_time_ = now();
            });

        timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&DataControl::onTimer, this));

        last_grid_start_time_ = now();
        grid_step_start_time_ = now();
        last_scan_time_ = now();
        last_pose_time_ = now();
    }

private:
    enum class Mode
    {
        Explore,
        Grid
    };
    struct StartPose
    {
        double x;
        double y;
        double yaw_rad;
    };
    struct GridPoint
    {
        double steer_rad;
        double speed_mps;
    };
    struct GridPair
    {
        size_t a;
        size_t b;
    };

private:
    // ---------- Main loop ----------
    void onTimer()
    {
        if (!has_scan_)
            return;
        if ((now() - last_scan_time_).seconds() > scan_timeout_sec_)
            return;

        updateRiskState();

        if (mode_ == Mode::Explore)
        {
            tryEnterGrid();
            const double steer = steerToOpenSpace(scan_);
            const double speed = risk_active_ ? explore_risk_speed_mps_ : explore_speed_mps_;
            publishDrive(steer, speed);
            std::cout << "Explore" << std::endl;
            std::cout << "Steer: " << steer << ", Speed: " << speed << std::endl;
            return;
        }

        runGrid();
    }

    // ---------- Drive ----------
    void publishDrive(double steer_rad, double speed_mps)
    {
        steer_rad = clamp(steer_rad, -steer_limit_rad_, steer_limit_rad_);
        last_cmd_speed_mps_ = speed_mps;

        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = now();
        cmd.header.frame_id = "base_link";
        cmd.drive.steering_angle = steer_rad;
        cmd.drive.speed = speed_mps;
        cmd.drive.acceleration = 0.0;
        drive_pub_->publish(cmd);
    }

    double currentSpeedMps() const
    {
        if (has_odom_)
            return std::max(0.0, static_cast<double>(odom_.twist.twist.linear.x));
        return std::max(0.0, last_cmd_speed_mps_);
    }

    // ---------- Explore steering ----------
    double steerToOpenSpace(const sensor_msgs::msg::LaserScan &scan) const
    {
        if (scan.angle_increment <= 0.0 || scan.ranges.empty())
            return 0.0;

        const double fov_half = deg2rad(explore_fov_half_deg_);
        const double win_half = deg2rad(explore_window_half_deg_);

        const double right = std::max(-fov_half, static_cast<double>(scan.angle_min));
        const double left = std::min(fov_half, static_cast<double>(scan.angle_max));
        if (right > left)
            return 0.0;

        const int i0 = clampIndex(angleToIndex(scan, right), scan.ranges.size());
        const int i1 = clampIndex(angleToIndex(scan, left), scan.ranges.size());
        int lo = std::min(i0, i1), hi = std::max(i0, i1);

        const int win_half_n = std::max(1, static_cast<int>(std::round(win_half / scan.angle_increment)));

        double best_score = -1.0;
        int best_i = (lo + hi) / 2;

        for (int i = lo; i <= hi; ++i)
        {
            const int a = std::max(lo, i - win_half_n);
            const int b = std::min(hi, i + win_half_n);

            double sum = 0.0;
            int cnt = 0;
            for (int j = a; j <= b; ++j)
            {
                const float r = scan.ranges[static_cast<size_t>(j)];
                if (!isValidRange(scan, r))
                    continue;
                sum += r;
                cnt++;
            }

            if (cnt < (b - a + 1) / 3)
                continue;

            const double score = sum / std::max(1, cnt);
            if (score > best_score)
            {
                best_score = score;
                best_i = i;
            }
        }

        return static_cast<double>(scan.angle_min) + best_i * static_cast<double>(scan.angle_increment);
    }

    // ---------- Grid safety check ----------
    bool isFanClear(const sensor_msgs::msg::LaserScan &scan, double steer_rad, double speed_mps) const
    {
        if (scan.angle_increment <= 0.0 || scan.ranges.empty())
            return false;

        const double need_m =
            speed_mps * grid_hold_sec_ * grid_horizon_mul_ + grid_margin_m_;

        const double center = deg2rad(fan_center_gain_deg_per_rad_ * steer_rad);
        const double half = deg2rad(fan_base_half_deg_ + fan_half_gain_deg_per_rad_ * std::fabs(steer_rad));

        const double right = std::max(center - half, static_cast<double>(scan.angle_min));
        const double left = std::min(center + half, static_cast<double>(scan.angle_max));
        if (right > left)
            return false;

        int i0 = clampIndex(angleToIndex(scan, right), scan.ranges.size());
        int i1 = clampIndex(angleToIndex(scan, left), scan.ranges.size());
        int lo = std::min(i0, i1), hi = std::max(i0, i1);

        int valid = 0;
        for (int i = lo; i <= hi; ++i)
        {
            const float r = scan.ranges[static_cast<size_t>(i)];
            if (!isValidRange(scan, r))
                continue;
            valid++;
            if (static_cast<double>(r) < need_m)
                return false;
        }
        return valid >= (hi - lo + 1) / 3;
    }

    // ---------- Grid (pair-of-pairs) ----------
    void tryEnterGrid()
    {
        if (grid_pairs_.empty() || grid_points_.empty())
            return;
        if (pairs_done_ >= grid_pairs_.size())
            return;
        if ((now() - last_grid_start_time_).seconds() < grid_cooldown_sec_)
            return;
        if (risk_active_)
            return;

        auto idx = findNextRunnablePair(grid_pair_index_);
        if (!idx)
            return;

        grid_pair_index_ = *idx;
        run_a_step_ = true;
        mode_ = Mode::Grid;
        grid_step_start_time_ = now();
        last_grid_start_time_ = now();
    }

    void runGrid()
    {
        if (pairs_done_ >= grid_pairs_.size())
        {
            mode_ = Mode::Explore;
            return;
        }

        if (risk_active_)
        {
            mode_ = Mode::Explore; // progress kept
            return;
        }

        // Skip already-done pairs quickly
        if (pair_done_[grid_pair_index_])
        {
            auto idx = findNextRunnablePair((grid_pair_index_ + 1) % grid_pairs_.size());
            if (!idx)
            {
                mode_ = Mode::Explore;
                return;
            }
            grid_pair_index_ = *idx;
            run_a_step_ = true;
            grid_step_start_time_ = now();
            return;
        }

        const GridPair &pair = grid_pairs_[grid_pair_index_];
        const GridPoint &p = run_a_step_ ? grid_points_[pair.a] : grid_points_[pair.b];

        // If blocked: drop back to Explore and advance index to avoid getting stuck
        if (!isFanClear(scan_, p.steer_rad, p.speed_mps))
        {
            mode_ = Mode::Explore;
            grid_pair_index_ = (grid_pair_index_ + 1) % grid_pairs_.size();
            run_a_step_ = true;
            return;
        }

        publishDrive(p.steer_rad, p.speed_mps);
        std::cout << "Grid" << std::endl;
        std::cout << "Steer: " << p.steer_rad << ", Speed: " << p.speed_mps << std::endl;

        if ((now() - grid_step_start_time_).seconds() < grid_hold_sec_)
            return;

        grid_step_start_time_ = now();

        if (run_a_step_)
        {
            run_a_step_ = false;
            return;
        } // A -> B

        // B done => mark pair done
        if (!pair_done_[grid_pair_index_])
        {
            pair_done_[grid_pair_index_] = 1;
            pairs_done_++;
        }

        auto next = findNextRunnablePair((grid_pair_index_ + 1) % grid_pairs_.size());
        if (!next)
        {
            mode_ = Mode::Explore;
            return;
        }

        grid_pair_index_ = *next;
        run_a_step_ = true;
    }

    std::optional<size_t> findNextRunnablePair(size_t start) const
    {
        if (grid_pairs_.empty())
            return std::nullopt;

        const size_t N = grid_pairs_.size();
        for (size_t k = 0; k < N; ++k)
        {
            const size_t idx = (start + k) % N;
            if (pair_done_[idx])
                continue;

            const auto &pair = grid_pairs_[idx];
            const auto &a = grid_points_[pair.a];
            const auto &b = grid_points_[pair.b];

            if (!isFanClear(scan_, a.steer_rad, a.speed_mps))
                continue;
            if (require_both_clear_ && !isFanClear(scan_, b.steer_rad, b.speed_mps))
                continue;

            return idx;
        }
        return std::nullopt;
    }

    // ---------- Risk (TTC) ----------
    void updateRiskState()
    {
        min_ttc_sec_ = computeMinTtc(scan_, currentSpeedMps());

        if (min_ttc_sec_ < ttc_enter_sec_)
        {
            risk_active_ = true;
            risk_exit_start_.reset();
            return;
        }

        if (!risk_active_)
            return;

        if (min_ttc_sec_ > ttc_exit_sec_)
        {
            if (!risk_exit_start_)
                risk_exit_start_ = now();
            if ((now() - *risk_exit_start_).seconds() >= ttc_exit_hold_sec_)
            {
                risk_active_ = false;
                risk_exit_start_.reset();
            }
        }
        else
        {
            risk_exit_start_.reset();
        }
    }

    double computeMinTtc(const sensor_msgs::msg::LaserScan &scan, double speed_mps) const
    {
        if (speed_mps <= 0.1)
            return std::numeric_limits<double>::infinity();

        double best = std::numeric_limits<double>::infinity();
        double angle = scan.angle_min;

        for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment)
        {
            const float r = scan.ranges[i];
            if (!isValidRange(scan, r))
                continue;

            const double closing = speed_mps * std::cos(angle);
            if (closing <= 1e-3)
                continue;

            best = std::min(best, static_cast<double>(r) / closing);
        }
        return best;
    }

    // ---------- Grid builders ----------
    void buildGridPoints()
    {
        grid_points_.clear();
        grid_points_.reserve(left_steer_grid_rad_.size() * speed_grid_mps_.size());
        for (double steer : left_steer_grid_rad_)
            for (double speed : speed_grid_mps_)
                grid_points_.push_back(GridPoint{steer, speed});
    }

    void buildGridPairs()
    {
        grid_pairs_.clear();
        const size_t n = grid_points_.size();
        grid_pairs_.reserve(n * n);
        for (size_t i = 0; i < n; ++i)
            for (size_t j = 0; j < n; ++j)
                grid_pairs_.push_back(GridPair{i, j});
    }

    // ---------- Small helpers ----------
    static double deg2rad(double deg) { return deg * M_PI / 180.0; }

    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(hi, v));
    }

    static bool isValidRange(const sensor_msgs::msg::LaserScan &scan, float r)
    {
        return std::isfinite(r) && r > scan.range_min && r < scan.range_max;
    }

    static int clampIndex(int i, size_t n)
    {
        if (n == 0)
            return 0;
        return std::max(0, std::min(i, static_cast<int>(n - 1)));
    }

    static int angleToIndex(const sensor_msgs::msg::LaserScan &scan, double angle)
    {
        const double a = std::min(std::max(angle, static_cast<double>(scan.angle_min)),
                                  static_cast<double>(scan.angle_max));
        return static_cast<int>(std::round((a - scan.angle_min) / scan.angle_increment));
    }

private:
    // ROS
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Messages
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::LaserScan scan_;
    bool has_odom_{false};
    bool has_scan_{false};

    // Topics
    std::string odom_topic_;
    std::string scan_topic_;
    std::string drive_topic_;

    // Explore
    double explore_speed_mps_{1.0};
    double explore_risk_speed_mps_{1.0};
    double last_cmd_speed_mps_{0.0};
    double explore_fov_half_deg_{90.0};
    double explore_window_half_deg_{7.0};
    double steer_limit_rad_{0.42};

    // Grid timing
    double grid_hold_sec_{1.0};
    double grid_cooldown_sec_{3.0};

    // Grid safety fan
    double grid_horizon_mul_{2.0};
    double grid_margin_m_{1.0};
    double fan_base_half_deg_{10.0};
    double fan_half_gain_deg_per_rad_{25.0};
    double fan_center_gain_deg_per_rad_{30.0};
    bool require_both_clear_{true};

    // Scan freshness
    double scan_timeout_sec_{0.2};

    // Risk (TTC)
    double ttc_enter_sec_{0.7};
    double ttc_exit_sec_{1.2};
    double ttc_exit_hold_sec_{0.5};
    double min_ttc_sec_{std::numeric_limits<double>::infinity()};
    bool risk_active_{false};
    std::optional<rclcpp::Time> risk_exit_start_;

    // Start Pose (x, y, yaw_rad)
    StartPose left_start_pose_1_{-2.0, -0.2, -1.6};
    StartPose left_start_pose_2_{2.0, -8.5, 1.6};
    StartPose right_start_pose_1_{2.0, -0.2, -1.6};
    StartPose right_start_pose_2_{-2.0, -8.5, 1.6};

    // Grid definition
    std::vector<double> left_steer_grid_rad_;
    std::vector<double> right_steer_grid_rad_;
    std::vector<double> speed_grid_mps_;
    std::vector<GridPoint> grid_points_;
    std::vector<GridPair> grid_pairs_;

    // Grid progress
    std::vector<uint8_t> pair_done_;
    size_t pairs_done_{0};
    size_t grid_pair_index_{0};
    bool run_a_step_{true};

    // State
    Mode mode_{Mode::Explore};
    rclcpp::Time grid_step_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_grid_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataControl>());
    rclcpp::shutdown();
    return 0;
}
