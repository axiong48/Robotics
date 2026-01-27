#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <cstdlib>
#include <ctime>
#include <vector>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

enum class RobotState
{
    INIT,
    CHECK_HUMAN_1,
    CHECK_HUMAN_2,
    RANDOM_SEARCH
};


class MoverNode : public rclcpp::Node
{
public:
    MoverNode() : Node("moverobo"),
                  map_received_(false),
                  current_state_(RobotState::INIT)
    {
        srand(time(NULL));

        // QoS for map
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        map_qos.transient_local();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos,
            std::bind(&MoverNode::map_cb, this, std::placeholders::_1));

        this->nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Waiting for the 'navigate_to_pose' action server...");
        if (!this->nav_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Action server is available.");
    }

    void run()
    {
        while (rclcpp::ok())
        {
            // Process any pending callbacks (like map)
            rclcpp::spin_some(this->get_node_base_interface());

            switch (current_state_)
            {
            case RobotState::INIT:
                RCLCPP_INFO_ONCE(this->get_logger(), "State: INIT. Waiting for map...");
                if (map_received_)
                {
                    RCLCPP_INFO(this->get_logger(), "Map received. Proceeding to check Human 1.");
                    current_state_ = RobotState::CHECK_HUMAN_1;
                }
                break;

            case RobotState::CHECK_HUMAN_1:
                RCLCPP_INFO(this->get_logger(), "State: CHECK_HUMAN_1. Navigating to (1.0, -1.0)...");

                send_goal_and_wait(1.0, -1.0, 1.0);
                RCLCPP_INFO(this->get_logger(), "Finished checking Human 1. Proceeding to check Human 2.");
                current_state_ = RobotState::CHECK_HUMAN_2;
                break;

            case RobotState::CHECK_HUMAN_2:
                RCLCPP_INFO(this->get_logger(), "State: CHECK_HUMAN_2. Navigating to (-12.0, 15.0)...");

                send_goal_and_wait(-12.0, 15.0, 1.0); 
                RCLCPP_INFO(this->get_logger(), "Finished checking Human 2. Proceeding to random search.");
                
                current_state_ = RobotState::RANDOM_SEARCH;
                break;

            case RobotState::RANDOM_SEARCH:
                RCLCPP_INFO_ONCE(this->get_logger(), "State: RANDOM_SEARCH. Starting random exploration...");

                send_random_goal_and_wait();
                RCLCPP_INFO(this->get_logger(), "Random goal complete. Finding next random goal...");
                // The loop repeats
                break;
            }

            // Sleep briefly to prevent this loop from spinning too fast if a state fails
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
    }

private:
    // Variables
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;

    nav_msgs::msg::OccupancyGrid map_data_;
    bool map_received_;
    std::vector<size_t> free_cells_indices_; // Stores valid free cells from the map
    RobotState current_state_;

    // Callbacks

    void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (map_received_)
            return;
        RCLCPP_INFO(this->get_logger(), "Map received! Processing to find free cells...");
        map_data_ = *msg;
        free_cells_indices_.clear();

        for (size_t i = 0; i < map_data_.data.size(); ++i)
        {
            if (map_data_.data[i] == 0) // 0 == nav_msgs::msg::OccupancyGrid::FREE_SPACE
            {
                free_cells_indices_.push_back(i);
            }
        }

        if (free_cells_indices_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Map received, but no free cells (value 0) were found!");
        }
        else
        {
            map_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Map processed. Found %zu free cells.", free_cells_indices_.size());
        }
    }

    // Goal Sending Functions

    // random free cell from the map
    void send_random_goal_and_wait()
    {
        if (free_cells_indices_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot send random goal: no free cells found in map.");
            return;
        }

        size_t random_vector_index = rand() % free_cells_indices_.size();

        // Get corresponding index in the map data array
        size_t map_index = free_cells_indices_[random_vector_index];

        // Convert 1D map index into 2D cell coordinates
        int col = map_index % map_data_.info.width;
        int row = map_index / map_data_.info.width;

        // Convert cell (col, row) to world coordinates
        double x = (col + 0.5) * map_data_.info.resolution + map_data_.info.origin.position.x;
        double y = (row + 0.5) * map_data_.info.resolution + map_data_.info.origin.position.y;

        send_goal_and_wait(x, y, 1.0);
    }


    void send_goal_and_wait(double x, double y, double w)
    {
        if (!nav_action_client_->action_server_is_ready())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server is not ready. Skipping goal.");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = w;

        RCLCPP_INFO(this->get_logger(), "Sending goal: [x: %.2f, y: %.2f]", x, y);

        // Send the goal
        auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, std::chrono::seconds(10)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Send goal call failed.");
            return;
        }

        GoalHandleNavigateToPose::SharedPtr goal_handle = future_goal_handle.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
            return;
        }

        // Wait for the result
        auto future_result = nav_action_client_->async_get_result(goal_handle);
        RCLCPP_INFO(this->get_logger(), "Waiting for navigation result...");

        // Block this function by spinning until the future is complete
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::minutes(5)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Get result call failed.");
            return;
        }

        // Check the result
        auto result_wrapper = future_result.get();
        if (result_wrapper.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Goal failed or was canceled with code: %d", static_cast<int>(result_wrapper.code));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoverNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}