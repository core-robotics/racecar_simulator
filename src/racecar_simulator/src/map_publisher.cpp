#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>

class MapPublisher : public rclcpp::Node {
public:
  MapPublisher() : Node("map_publisher") {
    declare_parameter<std::string>("map_img_file_path", "map.pgm");
    declare_parameter<std::string>("map_yaml_file_path", "map.yaml");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("obstacle_radius_m", 0.0);
    get_parameter("map_img_file_path", pgm_path_);
    get_parameter("map_yaml_file_path", yaml_path_);
    get_parameter("frame_id", frame_id_);
    get_parameter("obstacle_radius_m", obstacle_radius_m_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 10, std::bind(&MapPublisher::pointCallback, this, std::placeholders::_1));

    loadMap();
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&MapPublisher::publishMap, this));
  }

private:
  void loadMap() {
    YAML::Node y = YAML::LoadFile(yaml_path_);
    double res = y["resolution"].as<double>();
    auto origin = y["origin"].as<std::vector<double>>();
    int w,h,maxv; std::ifstream f(pgm_path_, std::ios::binary);
    std::string ln; f >> ln >> w >> h >> maxv; f.get();
    std::vector<uint8_t> pix(w*h); f.read((char*)pix.data(), w*h);

    map_msg_.info.resolution = res;
    map_msg_.info.width = w; map_msg_.info.height = h;
    map_msg_.info.origin.position.x = origin[0];
    map_msg_.info.origin.position.y = origin[1];
    map_msg_.header.frame_id = frame_id_;
    map_msg_.data.resize(w*h, -1);

    for(int y=0;y<h;y++)for(int x=0;x<w;x++){
      uint8_t p = pix[(h-1-y)*w+x];
      map_msg_.data[y*w+x] = (p<128)?100:0;
    }
  }

  void publishMap() {
    std::scoped_lock lk(mtx_);
    map_msg_.header.stamp = now();
    map_pub_->publish(map_msg_);
  }

  bool worldToMap(double x, double y, int &ix, int &iy) {
    double res=map_msg_.info.resolution, ox=map_msg_.info.origin.position.x, oy=map_msg_.info.origin.position.y;
    ix = (int)((x-ox)/res); iy = (int)((y-oy)/res);
    return ix>=0 && iy>=0 && ix<(int)map_msg_.info.width && iy<(int)map_msg_.info.height;
  }

  void paintDisk(int cx,int cy,int r){
    int W=map_msg_.info.width,H=map_msg_.info.height;
    for(int dy=-r;dy<=r;dy++)for(int dx=-r;dx<=r;dx++){
      int x=cx+dx,y=cy+dy; if(x<0||y<0||x>=W||y>=H)continue;
      if(dx*dx+dy*dy<=r*r) map_msg_.data[y*W+x]=100;
    }
  }

  void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    std::scoped_lock lk(mtx_);
    int ix,iy; if(!worldToMap(msg->point.x,msg->point.y,ix,iy))return;
    int r=std::round(obstacle_radius_m_/map_msg_.info.resolution);
    if(r<=0) map_msg_.data[iy*map_msg_.info.width+ix]=100;
    else paintDisk(ix,iy,r);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string pgm_path_, yaml_path_, frame_id_;
  double obstacle_radius_m_;
  nav_msgs::msg::OccupancyGrid map_msg_;
  std::mutex mtx_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  rclcpp::shutdown();
}
