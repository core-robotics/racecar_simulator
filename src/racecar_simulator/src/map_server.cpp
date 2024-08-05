#include "map_server.hpp"

// #include <string>
// #include <memory>

namespace map_server
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_server", options), map_available_(false)
{
    RCLCPP_INFO(get_logger(), "Creating");

    // Declare the node parameters
    declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
    declare_parameter("topic_name", "map");
    declare_parameter("frame_id", "map");

    getParameters();

    // Create a publisher using the QoS settings to emulate a ROS1 latched topic
    occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    sim_step_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(0.01 * 1000.0)),
        std::bind(&MapServer::loadMapCallback, this));
}

MapServer::~MapServer()
{
}

void MapServer::getParameters()
{
    std::string yaml_filename = get_parameter("yaml_filename").as_string();
    std::string topic_name = get_parameter("topic_name").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    // only try to load map if parameter was set
    if (yaml_filename.empty()) {
        RCLCPP_INFO(
        get_logger(),
        "yaml-filename parameter is empty");
    }
}

void MapServer::loadMapCallback()
{
    if (map_available_) {
        auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
        occ_pub_->publish(std::move(occ_grid));
    }
}

}

int main(int argc, char ** argv)
{
    std::string node_name("map_server");

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options{};
    auto node = std::make_shared<map_server::MapServer>(options);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}