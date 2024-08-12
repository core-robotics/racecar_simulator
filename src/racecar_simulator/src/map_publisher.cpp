#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Function to read the PGM file
bool read_pgm_file(const std::string &file_path, std::vector<int8_t> &map_data, int &width, int &height)
{
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "Failed to open PGM file: " << file_path << std::endl;
        return false;
    }

    std::string line;
    std::getline(file, line); // Read PGM format (P5)

    if (line != "P5")
    {
        std::cerr << "Invalid PGM file format: " << line << std::endl;
        return false;
    }

    // Skip comments
    while (std::getline(file, line))
    {
        if (line[0] != '#')
            break;
    }

    std::stringstream ss(line);
    ss >> width >> height;

    std::getline(file, line); // Read max grayscale value

    map_data.resize(width * height);

    file.read(reinterpret_cast<char *>(map_data.data()), map_data.size());

    file.close();
    return true;
}

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a node with parameters
    auto node = rclcpp::Node::make_shared("map_publisher");

    // Declare parameters
    node->declare_parameter<std::string>("pgm_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.pgm");
    node->declare_parameter<std::string>("yaml_file_path", "/home/a/racecar_simulator/src/racecar_simulator/maps/map7.yaml");

    // Get parameters
    std::string pgm_file_path, yaml_file_path;
    node->get_parameter("pgm_file_path", pgm_file_path);
    node->get_parameter("yaml_file_path", yaml_file_path);

    // Create a publisher for the OccupancyGrid
    auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    // Parse YAML file
    YAML::Node yaml_node = YAML::LoadFile(yaml_file_path);
    double resolution = yaml_node["resolution"].as<double>();
    std::vector<double> origin = yaml_node["origin"].as<std::vector<double>>();
    double occupied_thresh = yaml_node["occupied_thresh"].as<double>();
    double free_thresh = yaml_node["free_thresh"].as<double>();

    // Read PGM file
    std::vector<int8_t> pgm_data;
    int width, height;
    if (!read_pgm_file(pgm_file_path, pgm_data, width, height))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to load the map image!");
        return -1;
    }

    // Create OccupancyGrid message
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = "map";
    msg.info.resolution = resolution;
    msg.info.width = width;
    msg.info.height = height;
    msg.info.origin.position.x = origin[0];
    msg.info.origin.position.y = origin[1];
    msg.info.origin.position.z = origin[2];
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    // Convert the PGM data to occupancy values
    msg.data.resize(msg.info.width * msg.info.height);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            // 반전된 Y값 사용
            int reversed_y = height - 1 - y;
            uint8_t pixel = pgm_data[x + reversed_y * width];
            int index = x + y * width;

            if (pixel == 205)
            {
                msg.data[index] = -1; // Unknown
            }
            else if (pixel > occupied_thresh * 255)
            {
                msg.data[index] = 0; // Free
            }
            else if (pixel < free_thresh * 255)
            {
                msg.data[index] = 100; // Occupied
            }
            else
            {
                msg.data[index] = -1; // Unknown
            }
        }
    }

    // Create a time source and clock
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto time_source = std::make_shared<rclcpp::TimeSource>(node);
    time_source->attachClock(clock);

    // Loop to publish the map at a fixed rate
    rclcpp::WallRate loop_rate(10);
    while (rclcpp::ok())
    {
        msg.header.stamp = clock->now();
        map_pub->publish(msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
