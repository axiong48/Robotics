#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <atomic>
#include <cstring>
#include <thread>
#include <vector>
#include <algorithm>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <GeographicLib/UTMUPS.hpp>

using json = nlohmann::json;


const double PI = 3.14159265358979323846;


const std::vector<std::string> TREE_MODELS = {
    "model://orchard/orchard_tree_1.dae",
    "model://orchard/orchard_tree_3.dae",
    "model://orchard/orchard_tree_4.dae",
    "model://orchard/orchard_tree_5.dae"};

const std::string GROUND_URI = "model://orchard/orchard_world.dae";
const double TRUNK_HEIGHT = 3.0;
const double TRUNK_RADIUS = 0.25;
const double TREE_WIDTH_M = 5.0;
const double MODEL_WIDTH_M = 1.0;

class TcpDemuxNode : public rclcpp::Node
{
public:
  TcpDemuxNode() : rclcpp::Node("tcp_demux_node")
  {
    this->declare_parameter<int>("port", 12346);
    this->declare_parameter<bool>("payload_length_included", true);
    this->declare_parameter<std::string>("order", std::string("xml_then_json"));
    this->declare_parameter<bool>("expect_json", true);
    this->declare_parameter<int>("default_frame_size", 65536);
    this->declare_parameter<std::string>("mission_topic", std::string("/mission/xml"));
    this->declare_parameter<std::string>("orchard_topic", std::string("/orchard/tree_info_json"));
    this->declare_parameter<std::string>("output_path", "");

    mission_pub_ = this->create_publisher<std_msgs::msg::String>(
        this->get_parameter("mission_topic").as_string(), 1);
    orchard_pub_ = this->create_publisher<std_msgs::msg::String>(
        this->get_parameter("orchard_topic").as_string(), 1);

    std::srand(std::time(nullptr));

    server_thread_ = std::thread([this]() { this->server_loop(); });
  }

  ~TcpDemuxNode() override
  {
    running_ = false;
    if (server_thread_.joinable())
      server_thread_.join();
  }

private:
  double randomDouble(double min, double max)
  {
    return min + (double)rand() / RAND_MAX * (max - min);
  }
  // generates SDF for gazebo
  void generateSDF(const std::string &json_str)
  {
    try
    {
      auto tree_data = json::parse(json_str);
      if (tree_data.empty())
        return;

      RCLCPP_INFO(this->get_logger(), "Generating SDF for %zu trees...", tree_data.size());

      // Calculate Average Lat/Lon (Geometric Center)
      double sum_lat = 0.0, sum_lon = 0.0;
      for (const auto &t : tree_data)
      {
        sum_lat += t["lat"].get<double>();
        sum_lon += t["lon"].get<double>();
      }
      double anchor_lat_gps = sum_lat / tree_data.size();
      double anchor_lon_gps = sum_lon / tree_data.size();

      // Calculate Anchor UTM using GeographicLib
      int anchor_zone;
      bool anchor_northp;
      double anchor_easting, anchor_northing;

      try {
          GeographicLib::UTMUPS::Forward(anchor_lat_gps, anchor_lon_gps, anchor_zone, anchor_northp, anchor_easting, anchor_northing);
      } catch (const GeographicLib::GeographicErr& e) {
          RCLCPP_ERROR(this->get_logger(), "GeographicLib Anchor Conversion Failed: %s", e.what());
          return;
      }

      // To determine the outpath
      std::string output_path_param = this->get_parameter("output_path").as_string();
      std::string final_path = output_path_param.empty() ? "generated_orchard.sdf" : output_path_param;

      RCLCPP_INFO(this->get_logger(), "Calculated Anchor GPS: %.6f, %.6f (Zone %d%s)", 
                  anchor_lat_gps, anchor_lon_gps, anchor_zone, (anchor_northp ? "N" : "S"));
      RCLCPP_INFO(this->get_logger(), "Saving world to: %s", final_path.c_str());

      std::ofstream sdf(final_path);
      if (!sdf.is_open())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing.", final_path.c_str());
        return;
      }

      sdf << "<?xml version='1.0' encoding='ASCII'?>\n"
          << "<sdf version='1.4'>\n"
          << "  <world name='procedural_orchard'>\n"
          << "    <spherical_coordinates>\n"
          << "      <surface_model>EARTH_WGS84</surface_model>\n"
          << "      <latitude_deg>" << anchor_lat_gps << "</latitude_deg>\n"
          << "      <longitude_deg>" << anchor_lon_gps << "</longitude_deg>\n"
          << "      <elevation>0.0</elevation>\n"
          << "      <heading_deg>0</heading_deg>\n"
          << "    </spherical_coordinates>\n";

      sdf << R"(
    <physics type='ode'>
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='libgz-sim-physics-system.so'/>
    <plugin name='gz::sim::systems::UserCommands' filename='libgz-sim-user-commands-system.so'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='libgz-sim-scene-broadcaster-system.so'/>
    <plugin name='gz::sim::systems::Sensors' filename='libgz-sim-sensors-system.so'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name='gz::sim::systems::Imu' filename='libgz-sim-imu-system.so'/>
    <plugin name='gz::sim::systems::NavSat' filename='libgz-sim-navsat-system.so'/>
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>1</shadows>
      <sky><clouds><speed>12</speed></clouds></sky>
    </scene>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1.0</direction>
    </light>
    <model name='orchard_ground'>
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
        <collision name='col'><geometry><mesh><uri>)"
          << GROUND_URI << R"(</uri></mesh></geometry></collision>
        <visual name='vis'><geometry><mesh><uri>)"
          << GROUND_URI << R"(</uri></mesh></geometry></visual>
      </link>
    </model>
          )";

      // Generate Trees
      for (const auto &t : tree_data)
      {
        int idx = t["tree_index"].get<int>();
        double lat = t["lat"].get<double>();
        double lon = t["lon"].get<double>();

        // Convert Tree GPS to UTM using GeographicLib
        int tree_zone;
        bool tree_northp;
        double tree_easting, tree_northing;

        try {
            GeographicLib::UTMUPS::Forward(lat, lon, tree_zone, tree_northp, tree_easting, tree_northing);
        } catch (const GeographicLib::GeographicErr& e) {
            RCLCPP_ERROR(this->get_logger(), "GeographicLib Tree %d Conversion Failed: %s", idx, e.what());
            continue; 
        }

        // Calculate difference in meters from anchor origin
        double world_x = tree_easting - anchor_easting;
        double world_y = tree_northing - anchor_northing;

        // Randomize visuals
        double base_scale = TREE_WIDTH_M / MODEL_WIDTH_M;
        double random_factor = randomDouble(0.9, 1.1);
        double final_scale = base_scale * random_factor;
        double world_yaw = randomDouble(0, 2 * PI);

        int model_idx = std::rand() % TREE_MODELS.size();
        std::string selected_model = TREE_MODELS[model_idx];

        sdf << "    <model name='tree_" << idx << "'>\n"
            << "      <static>true</static>\n"
            << "      <pose>" << world_x << " " << world_y << " 0 0 0 " << world_yaw << "</pose>\n"
            << "      <link name='link'>\n"
            << "        <visual name='visual'>\n"
            << "          <geometry><mesh>\n"
            << "            <uri>" << selected_model << "</uri>\n"
            << "            <scale>" << final_scale << " " << final_scale << " " << final_scale << "</scale>\n"
            << "          </mesh></geometry>\n"
            << "        </visual>\n"
            << "        <collision name='trunk_collision'>\n"
            << "          <pose>0 0 " << (TRUNK_HEIGHT / 2.0) << " 0 0 0</pose>\n"
            << "          <geometry>\n"
            << "            <cylinder>\n"
            << "              <radius>" << TRUNK_RADIUS << "</radius>\n"
            << "              <length>" << TRUNK_HEIGHT << "</length>\n"
            << "            </cylinder>\n"
            << "          </geometry>\n"
            << "        </collision>\n"
            << "      </link>\n"
            << "    </model>\n";
      }

      sdf << "  </world>\n</sdf>";
      sdf.close();
      RCLCPP_INFO(this->get_logger(), "Successfully generated world file!");
    }
    catch (json::parse_error &e)
    {
      RCLCPP_ERROR(this->get_logger(), "JSON Parse Error: %s", e.what());
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "SDF Generation Error: %s", e.what());
    }
  }

  void server_loop()
  {
    int port = this->get_parameter("port").as_int();
    bool length_included = this->get_parameter("payload_length_included").as_bool();
    std::string order = this->get_parameter("order").as_string();
    bool expect_json = this->get_parameter("expect_json").as_bool();
    int default_frame_size = this->get_parameter("default_frame_size").as_int();

    while (rclcpp::ok() && running_)
    {
      int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
      if (server_fd < 0)
      {
        RCLCPP_FATAL(this->get_logger(), "socket() failed: %s", std::strerror(errno));
        return;
      }
      int opt = 1;
      if (::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0)
      {
        RCLCPP_FATAL(this->get_logger(), "setsockopt() failed: %s", std::strerror(errno));
        ::close(server_fd);
        return;
      }
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = INADDR_ANY;
      address.sin_port = htons(static_cast<uint16_t>(port));
      if (::bind(server_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
      {
        RCLCPP_FATAL(this->get_logger(), "bind() failed on port %d: %s", port, std::strerror(errno));
        ::close(server_fd);
        return;
      }
      if (::listen(server_fd, 1) < 0)
      {
        RCLCPP_FATAL(this->get_logger(), "listen() failed: %s", std::strerror(errno));
        ::close(server_fd);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "tcp_demux: Waiting on port %d...", port);
      socklen_t addrlen = sizeof(address);
      int client_fd = ::accept(server_fd, reinterpret_cast<sockaddr *>(&address), &addrlen);
      if (client_fd < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "accept() failed: %s", std::strerror(errno));
        ::close(server_fd);
        continue;
      }

      auto read_exact = [&](void *dst, size_t len) -> bool
      {
        size_t total = 0;
        char *ptr = static_cast<char *>(dst);
        while (total < len)
        {
          ssize_t n = ::read(client_fd, ptr + total, len - total);
          if (n > 0)
          {
            total += static_cast<size_t>(n);
          }
          else if (n == 0)
          {
            return false;
          }
          else
          {
            if (errno == EINTR)
              continue;
            return false;
          }
        }
        return true;
      };

      auto read_frame = [&](std::string &out) -> bool
      {
        uint32_t sz_n = 0;
        if (!read_exact(&sz_n, sizeof(sz_n)))
          return false;
        size_t sz = static_cast<size_t>(ntohl(sz_n));
        out.resize(sz);
        if (sz > 0 && !read_exact(&out[0], sz))
          return false;
        return true;
      };

      if (expect_json && length_included)
      {
        std::string first, second;
        bool ok1 = read_frame(first);
        bool ok2 = ok1 && read_frame(second);

        if (ok2)
        {
          std_msgs::msg::String xml_msg, json_msg;
          if (order == "json_then_xml")
          {
            json_msg.data = first;
            xml_msg.data = second;
          }
          else
          {
            xml_msg.data = first;
            json_msg.data = second;
          }

          mission_pub_->publish(xml_msg);
          orchard_pub_->publish(json_msg);
          generateSDF(json_msg.data);

          RCLCPP_INFO(this->get_logger(), "Processed payload and generated world.");
        }
      }

      ::close(client_fd);
      ::close(server_fd);
    }
  }

  std::atomic<bool> running_{true};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr orchard_pub_;
  std::thread server_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpDemuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}