#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <set>

class HumanDetectorNode : public rclcpp::Node
{
public:
    HumanDetectorNode() : Node("human_detector_node"),
                          static_map_received_(false),
                          costmap_received_(false)
    {
        // Parameters for ORIGINAL human locations
        this->declare_parameter<double>("human_1_orig_x", 1.00);
        this->declare_parameter<double>("human_1_orig_y", -1.00);
        this->declare_parameter<double>("human_2_orig_x", -12.00);
        this->declare_parameter<double>("human_2_orig_y", 15.00);

        human_1_orig_pose_.x = this->get_parameter("human_1_orig_x").as_double();
        human_1_orig_pose_.y = this->get_parameter("human_1_orig_y").as_double();
        human_2_orig_pose_.x = this->get_parameter("human_2_orig_x").as_double();
        human_2_orig_pose_.y = this->get_parameter("human_2_orig_y").as_double();

        RCLCPP_INFO(this->get_logger(), "Original Human 1 Pose: [x: %.2f, y: %.2f]",
                    human_1_orig_pose_.x, human_1_orig_pose_.y);
        RCLCPP_INFO(this->get_logger(), "Original Human 2 Pose: [x: %.2f, y: %.2f]",
                    human_2_orig_pose_.x, human_2_orig_pose_.y);

        // Subscribers

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        map_qos.transient_local();

        static_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos,
            std::bind(&HumanDetectorNode::static_map_cb, this, std::placeholders::_1));

        auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", costmap_qos,
            std::bind(&HumanDetectorNode::global_costmap_cb, this, std::placeholders::_1));

        // Timer for comparison loop differences
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HumanDetectorNode::compare_maps, this));

        RCLCPP_INFO(this->get_logger(), "Human Detector started. Waiting for maps...");
    }

private:
    // convert world x, y to 1D map array
    int worldToMap(double x, double y, const nav_msgs::msg::MapMetaData &info)
    {
        double x_cell = (x - info.origin.position.x) / info.resolution;
        double y_cell = (y - info.origin.position.y) / info.resolution;

        if (x_cell < 0 || x_cell >= info.width || y_cell < 0 || y_cell >= info.height)
        {
            return -1; // Out of bounds
        }

        int x_grid = static_cast<int>(x_cell);
        int y_grid = static_cast<int>(y_cell);

        return y_grid * info.width + x_grid;
    }

    geometry_msgs::msg::Point mapToWorld(int index, const nav_msgs::msg::MapMetaData &info)
    {
        int y_grid = index / info.width;
        int x_grid = index % info.width;

        geometry_msgs::msg::Point world_point;
        // Get center of the cell
        world_point.x = (x_grid + 0.5) * info.resolution + info.origin.position.x;
        world_point.y = (y_grid + 0.5) * info.resolution + info.origin.position.y;
        world_point.z = 0.0;
        return world_point;
    }

    // --- CALLBACKS ---

    void static_map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (static_map_received_)
            return;
        static_map_ = *msg;
        static_map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received static /map.");
    }

    void global_costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {

        global_costmap_ = *msg;
        if (!costmap_received_)
        {
            RCLCPP_INFO(this->get_logger(), "Received live /global_costmap/costmap.");
        }
        costmap_received_ = true;
    }

    void compare_maps()
    {

        if (!static_map_received_ || !costmap_received_)
        {
            return;
        }

        // check if compatible
        if (static_map_.info.width != global_costmap_.info.width ||
            static_map_.info.height != global_costmap_.info.height ||
            static_map_.info.resolution != global_costmap_.info.resolution)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Static map and costmap have different geometries! Cannot compare.");
            return;
        }

        // Check if original humans have moved
        check_original_location(human_1_orig_pose_, "Person 1 - Standing", human_1_moved_reported_);
        check_original_location(human_2_orig_pose_, "Person 2 - Walking", human_2_moved_reported_);

        // Find new obstacles
        find_new_obstacles();
    }

    void check_original_location(const geometry_msgs::msg::Point &pose, const std::string &name, bool &reported_flag)
    {
        if (reported_flag)
            return;

        int map_index = worldToMap(pose.x, pose.y, static_map_.info);
        if (map_index == -1)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Original pose for %s is out of map bounds.", name.c_str());
            return;
        }

        int static_val = static_map_.data[map_index];
        int live_val = global_costmap_.data[map_index];

        // If blueprint said "occupied" (>0) but the live map says "free" (0), it means the robot has scanned the area and confirmed the object is gone.
        // check for > 50 to be sure it was a solid obstacle.
        if (static_val > 50 && live_val == 0)
        {
            RCLCPP_FATAL(this->get_logger(), "-------------------------------------------------");
            RCLCPP_FATAL(this->get_logger(), "TASK 1: MOVEMENT DETECTED!");
            RCLCPP_FATAL(this->get_logger(), "  %s is NO LONGER at its original location.", name.c_str());
            RCLCPP_FATAL(this->get_logger(), "-------------------------------------------------");
            reported_flag = true;
        }
    }

    void find_new_obstacles()
    {
        // Loop through the entire map grid
        for (size_t i = 0; i < static_map_.data.size(); ++i)
        {
            int static_val = static_map_.data[i];
            int live_val = global_costmap_.data[i];

            if (static_val == 0 && live_val > 50)
            {
                // Check if we already reported this specific grid cell
                if (reported_new_obstacles_.find(i) == reported_new_obstacles_.end())
                {

                    geometry_msgs::msg::Point new_location = mapToWorld(i, static_map_.info);

                    RCLCPP_WARN(this->get_logger(), "================================================");
                    RCLCPP_WARN(this->get_logger(), "TASK 2: NEW OBSTACLE FOUND!");
                    RCLCPP_WARN(this->get_logger(), "  An unmapped object is at [x: %.2f, y: %.2f]",
                                new_location.x, new_location.y);
                    RCLCPP_WARN(this->get_logger(), "  (This could be the moved human or another object)");
                    RCLCPP_WARN(this->get_logger(), "================================================");

                    // Add to our set so we dont report it again
                    reported_new_obstacles_.insert(i);
                }
            }
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid static_map_;
    nav_msgs::msg::OccupancyGrid global_costmap_;

    bool static_map_received_;
    bool costmap_received_;

    geometry_msgs::msg::Point human_1_orig_pose_;
    geometry_msgs::msg::Point human_2_orig_pose_;

    bool human_1_moved_reported_ = false;
    bool human_2_moved_reported_ = false;

    // to store the *indices* of new obstacles we found
    std::set<int> reported_new_obstacles_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HumanDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
