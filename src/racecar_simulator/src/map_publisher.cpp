#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <cmath>
#include <mutex>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

class MapPublisher : public rclcpp::Node
{
public:
  MapPublisher() : Node("map_publisher")
  {
    declare_parameter<std::string>("map_img_file_path", "map.pgm");
    declare_parameter<std::string>("map_yaml_file_path", "map.yaml");
    declare_parameter<std::string>("race_line_file_path", "race_line.csv");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("obstacle_radius_m", 0.1);
    // declare_parameter<bool>("use_sim_time", false);

    get_parameter("map_img_file_path", img_path_);
    get_parameter("map_yaml_file_path", yaml_path_);
    get_parameter("race_line_file_path", race_line_path_);
    get_parameter("frame_id", frame_id_);
    get_parameter("obstacle_radius_m", obstacle_radius_m_);

    auto qos_map = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto qos_path = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&MapPublisher::publishAll, this));

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos_map);
    path_center_pub_ = create_publisher<nav_msgs::msg::Path>("center_path", qos_path);
    path_left_pub_ = create_publisher<nav_msgs::msg::Path>("left_boundary", qos_path);
    path_right_pub_ = create_publisher<nav_msgs::msg::Path>("right_boundary", qos_path);

    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", 10, std::bind(&MapPublisher::pointCallback, this, std::placeholders::_1));

    loadMap();
    loadRaceLineCSV(); // build center/left/right Path
  }

private:
  // --- Map loading (PGM/PNG) ---
  void loadMap()
  {
    auto y = YAML::LoadFile(yaml_path_);
    double res = y["resolution"].as<double>();
    auto origin = y["origin"].as<std::vector<double>>(); // [x,y,yaw]

    cv::Mat img = cv::imread(img_path_, cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
      RCLCPP_FATAL(get_logger(), "Failed to read image: %s", img_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    map_msg_.info.resolution = res;
    map_msg_.info.width = img.cols;
    map_msg_.info.height = img.rows;
    map_msg_.info.origin.position.x = origin[0];
    map_msg_.info.origin.position.y = origin[1];
    map_msg_.header.frame_id = frame_id_;
    map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height, -1);

    const int W = map_msg_.info.width, H = map_msg_.info.height;
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x)
      {
        uint8_t p = img.at<uint8_t>(H - 1 - y, x); // flip Y
        map_msg_.data[y * W + x] = (p < 128) ? 100 : 0;
      }
  }

  // --- CSV -> three Paths (center/left/right) ---
  static std::string lower_trim(std::string s)
  {
    auto issp = [](unsigned char c)
    { return std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [&](char c)
                                    { return !issp(c); }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [&](char c)
                         { return !issp(c); })
                .base(),
            s.end());
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
  }

  // Minimal, robust CSV → center/left/right Path
  void loadRaceLineCSV()
  {
    auto trim = [](std::string &s)
    {
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char c)
                                      { return !std::isspace(c); }));
      s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char c)
                           { return !std::isspace(c); })
                  .base(),
              s.end());
    };
    auto split = [&](std::string s)
    {
      std::replace(s.begin(), s.end(), ';', ','); // allow ';'
      std::stringstream ss(s);
      std::vector<std::string> v;
      std::string t;
      while (std::getline(ss, t, ','))
      {
        trim(t);
        v.push_back(t);
      }
      return v;
    };
    auto getline_clean = [&](std::istream &ifs, std::string &out) -> bool
    {
      while (std::getline(ifs, out))
      {
        if (!out.empty() && out.back() == '\r')
          out.pop_back(); // CRLF
        if (out.size() >= 3 && (uint8_t)out[0] == 0xEF && (uint8_t)out[1] == 0xBB && (uint8_t)out[2] == 0xBF)
          out.erase(0, 3); // BOM
        std::string t = out;
        trim(t);
        if (t.empty() || t[0] == '#')
        {
          if (!t.empty())
            return true;
          else
            continue;
        }
        return true;
      }
      return false;
    };

    std::ifstream f(race_line_path_);
    if (!f.is_open())
    {
      RCLCPP_WARN(get_logger(), "CSV open fail: %s", race_line_path_.c_str());
      return;
    }

    // header (accepts "# x_m, y_m, w_tr_right_m, w_tr_left_m")
    std::string line;
    if (!getline_clean(f, line))
    {
      RCLCPP_WARN(get_logger(), "CSV empty");
      return;
    }
    {
      std::string t = line;
      trim(t);
      if (!t.empty() && t[0] == '#')
        line.erase(line.find('#'), 1);
    }
    auto headers = split(line);
    auto find_idx = [&](std::initializer_list<const char *> names) -> int
    {
      for (auto n : names)
      {
        for (size_t i = 0; i < headers.size(); ++i)
        {
          std::string h = headers[i];
          std::string nn = n;
          trim(h);
          trim(nn);
          std::transform(h.begin(), h.end(), h.begin(), ::tolower);
          std::transform(nn.begin(), nn.end(), nn.begin(), ::tolower);
          if (h == nn)
            return (int)i;
        }
      }
      return -1;
    };

    int ix = find_idx({"x_m", "x", "x_map", "map_x"});
    int iy = find_idx({"y_m", "y", "y_map", "map_y"});
    int ip = find_idx({"psi_rad", "psi", "yaw", "heading_rad", "theta"}); // optional
    int il = find_idx({"w_tr_left_m", "w_left_m", "left_width_m", "w_tr_left"});
    int ir = find_idx({"w_tr_right_m", "w_right_m", "right_width_m", "w_tr_right"});
    if (ix < 0 || iy < 0)
    {
      RCLCPP_WARN(get_logger(), "Need X/Y columns");
      return;
    }

    struct Row
    {
      double x{}, y{}, psi{}, wl{}, wr{};
      bool hp = false, hl = false, hr = false;
    };
    std::vector<Row> rows;

    // data
    while (getline_clean(f, line))
    {
      std::string t = line;
      std::string tt = t;
      trim(tt);
      if (!tt.empty() && tt[0] == '#')
        continue;
      auto tok = split(t);
      auto getd = [&](int i, double &o)
      { if(i<0||i>=(int)tok.size()||tok[i].empty())return false; try{o=std::stod(tok[i]);return true;}catch(...){return false;} };
      Row r{};
      r.hp = getd(ip, r.psi);
      r.hl = getd(il, r.wl);
      r.hr = getd(ir, r.wr);
      if (!getd(ix, r.x) || !getd(iy, r.y))
        continue;
      rows.push_back(r);
    }
    if (rows.size() < 2)
    {
      RCLCPP_WARN(get_logger(), "Too few rows");
      return;
    }

    // yaw from x,y (prev→curr). Minimal.
    const double EPS = 1e-6;
    for (size_t i = 0; i < rows.size(); ++i)
    {
      double dx = 0, dy = 0;
      if (i == 0 && rows.size() > 1)
      {
        dx = rows[1].x - rows[0].x;
        dy = rows[1].y - rows[0].y;
      }
      else if (i > 0)
      {
        dx = rows[i].x - rows[i - 1].x;
        dy = rows[i].y - rows[i - 1].y;
      }
      else
      {
        rows[i].psi = 0.0;
        continue;
      }
      rows[i].psi = (std::hypot(dx, dy) > EPS) ? std::atan2(dy, dx) : (i ? rows[i - 1].psi : 0.0);
    }
    // unwrap to avoid ±π jumps
    for (size_t i = 1; i < rows.size(); ++i)
    {
      double d = rows[i].psi - rows[i - 1].psi;
      while (d > M_PI)
        d -= 2 * M_PI;
      while (d < -M_PI)
        d += 2 * M_PI;
      rows[i].psi = rows[i - 1].psi + d;
    }
    // widths default if CSV lacks them
    for (auto &r : rows)
    {
      if (!r.hl)
        r.wl = 0.0;
      if (!r.hr)
        r.wr = 0.0;
    }

    // build paths
    nav_msgs::msg::Path pc, pl, pr;
    pc.header.frame_id = frame_id_;
    pl.header.frame_id = frame_id_;
    pr.header.frame_id = frame_id_;
    pc.poses.reserve(rows.size());
    pl.poses.reserve(rows.size());
    pr.poses.reserve(rows.size());

    for (auto &r : rows)
    {
      double nx = -std::sin(r.psi), ny = std::cos(r.psi); // left normal
      geometry_msgs::msg::PoseStamped c, lft, rgt;
      c.header.frame_id = frame_id_;
      lft.header.frame_id = frame_id_;
      rgt.header.frame_id = frame_id_;
      c.pose.position.x = r.x;
      c.pose.position.y = r.y;
      lft.pose.position.x = r.x + nx * r.wl;
      lft.pose.position.y = r.y + ny * r.wl;
      rgt.pose.position.x = r.x - nx * r.wr;
      rgt.pose.position.y = r.y - ny * r.wr;
      c.pose.orientation.z = std::sin(r.psi * 0.5);
      c.pose.orientation.w = std::cos(r.psi * 0.5);
      lft.pose.orientation = c.pose.orientation;
      rgt.pose.orientation = c.pose.orientation;
      pc.poses.push_back(c);
      pl.poses.push_back(lft);
      pr.poses.push_back(rgt);
    }

    std::scoped_lock lk(mtx_);
    path_center_ = std::move(pc);
    path_left_ = std::move(pl);
    path_right_ = std::move(pr);
  }

  // --- Publishers ---
  void publishAll()
  {
    std::scoped_lock lk(mtx_);
    // map
    map_msg_.header.stamp = this->get_clock()->now();
    map_pub_->publish(map_msg_);

    // paths
    path_center_.header.stamp = this->get_clock()->now();
    path_left_.header.stamp = this->get_clock()->now();
    path_right_.header.stamp = this->get_clock()->now();

    path_center_.header.frame_id = frame_id_;
    path_left_.header.frame_id = frame_id_;
    path_right_.header.frame_id = frame_id_;

    path_center_pub_->publish(path_center_);
    path_left_pub_->publish(path_left_);
    path_right_pub_->publish(path_right_);
  }

  // --- Helpers for obstacle stamping on the grid ---
  bool worldToMap(double x, double y, int &ix, int &iy)
  {
    double r = map_msg_.info.resolution;
    double ox = map_msg_.info.origin.position.x, oy = map_msg_.info.origin.position.y;
    ix = int((x - ox) / r);
    iy = int((y - oy) / r);
    return ix >= 0 && iy >= 0 &&
           ix < int(map_msg_.info.width) && iy < int(map_msg_.info.height);
  }

  void paintDisk(int cx, int cy, int r)
  {
    int W = map_msg_.info.width, H = map_msg_.info.height;
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx)
      {
        int x = cx + dx, y = cy + dy;
        if (x < 0 || y < 0 || x >= W || y >= H)
          continue;
        if (dx * dx + dy * dy <= r * r)
          map_msg_.data[y * W + x] = 100;
      }
  }

  void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    std::scoped_lock lk(mtx_);
    int ix, iy;
    if (!worldToMap(msg->point.x, msg->point.y, ix, iy))
      return;
    int r = std::lround(obstacle_radius_m_ / map_msg_.info.resolution);
    if (r <= 0)
      map_msg_.data[iy * map_msg_.info.width + ix] = 100;
    else
    {
      RCLCPP_INFO(get_logger(), "\nCreate obstacle at [%f, %f]", msg->point.x, msg->point.y);
      paintDisk(ix, iy, r);
    }
  }

  // --- Members ---
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_center_pub_, path_left_pub_, path_right_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string img_path_, yaml_path_, race_line_path_, frame_id_;
  double obstacle_radius_m_{};

  nav_msgs::msg::OccupancyGrid map_msg_;
  nav_msgs::msg::Path path_center_, path_left_, path_right_;
  std::mutex mtx_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  rclcpp::shutdown();
  return 0;
}
