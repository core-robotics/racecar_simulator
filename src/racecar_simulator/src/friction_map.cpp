#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <yaml-cpp/yaml.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>

#include <filesystem>
#include <cmath>
#include <limits>
#include <string>
#include <chrono>

class FrictionGridMapPublisher final : public rclcpp::Node
{
public:
    FrictionGridMapPublisher() : Node("friction_gridmap_publisher")
    {
        yaml_path_ = declare_parameter<std::string>("friction_yaml", "friction.yaml");
        topic_ = declare_parameter<std::string>("topic", "/friction_gridmap");
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        publish_hz_ = declare_parameter<double>("publish_hz", 1.0);
        flip_y_ = declare_parameter<bool>("flip_y", false);
        apply_yaw_ = declare_parameter<bool>("apply_yaw", false);

        publisher_ = create_publisher<grid_map_msgs::msg::GridMap>(topic_, rclcpp::QoS(1).transient_local());

        buildMapFromYaml();

        const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_hz_));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]
            { publishOnce(); });
    }

private:
    static std::string resolveRelativePath(const std::string &yaml_path, const std::string &path)
    {
        namespace fs = std::filesystem;
        fs::path p(path);
        if (p.is_absolute())
            return p.string();
        fs::path base = fs::path(yaml_path).parent_path();
        if (base.empty())
            base = ".";
        return (base / p).lexically_normal().string();
    }

    static grid_map::Position computeCenter(
        double origin_x, double origin_y, double yaw, double size_x, double size_y, bool apply_yaw)
    {
        const double hx = 0.5 * size_x;
        const double hy = 0.5 * size_y;

        if (!apply_yaw)
            return {origin_x + hx, origin_y + hy};

        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        return {origin_x + hx * c - hy * s, origin_y + hx * s + hy * c};
    }

    void buildMapFromYaml()
    {
        const YAML::Node cfg = YAML::LoadFile(yaml_path_);

        const std::string image_path = resolveRelativePath(yaml_path_, cfg["image"].as<std::string>());
        const double resolution = cfg["resolution"].as<double>();

        const auto origin = cfg["origin"];
        const double origin_x = origin[0].as<double>();
        const double origin_y = origin[1].as<double>();
        const double yaw = origin[2].as<double>();

        mu_min_ = cfg["mu_min"] ? cfg["mu_min"].as<double>() : 0.0;
        mu_max_ = cfg["mu_max"] ? cfg["mu_max"].as<double>() : 1.0;
        unknown_pixel_ = cfg["unknown_pixel"] ? cfg["unknown_pixel"].as<int>() : -1;

        const cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if (image.empty())
            throw std::runtime_error("Failed to read image: " + image_path);

        const int width = image.cols;
        const int height = image.rows;

        const double size_x = width * resolution;
        const double size_y = height * resolution;

        const auto center = computeCenter(origin_x, origin_y, yaw, size_x, size_y, apply_yaw_);

        map_.setFrameId(frame_id_);
        map_.setGeometry(grid_map::Length(size_x, size_y), resolution, center);
        map_.add(layer_);
        map_[layer_].setConstant(std::numeric_limits<float>::quiet_NaN());

        for (int r = 0; r < height; ++r)
        {
            const int map_r = flip_y_ ? (height - 1 - r) : r;
            for (int c = 0; c < width; ++c)
            {
                const int pixel = static_cast<int>(image.at<uint8_t>(r, c));
                if (unknown_pixel_ >= 0 && pixel == unknown_pixel_)
                    continue;

                const double t = static_cast<double>(pixel) / 255.0;
                const double mu = mu_min_ + t * (mu_max_ - mu_min_);
                const int x = width - 1 - c;
                map_.at(layer_, grid_map::Index(x, map_r)) = static_cast<float>(mu);
            }
        }
    }

    void publishOnce()
    {
        map_.setTimestamp(get_clock()->now().nanoseconds());
        publisher_->publish(*grid_map::GridMapRosConverter::toMessage(map_));
    }

    const std::string layer_{"friction"};

    std::string yaml_path_;
    std::string topic_;
    std::string frame_id_;

    double publish_hz_{1.0};
    bool flip_y_{true};
    bool apply_yaw_{true};

    double mu_min_{0.0};
    double mu_max_{1.0};
    int unknown_pixel_{-1};

    grid_map::GridMap map_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrictionGridMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
