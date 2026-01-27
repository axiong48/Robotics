#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "navigation/navigation.hpp"

using namespace std::chrono_literals;

class testnavigator : public rclcpp::Node
{
public:
    testnavigator(bool debug = false, bool verbose = false)
        : rclcpp::Node("testnavigator_node"),
          navigator_(debug, false)
    {
    }

    int run()
    {
        try
        {
            setInitialPose("map", -2.0, 0.0, 0.0);

            std::cout << "Waiting for Nav2 to become active...\n";
            navigator_.WaitUntilNav2Active();
            std::cout << "Nav2 is active.\n";

            auto goal = makePose(1.5, 0.5, 0.0, 0.0, "map");
            std::cout << "Requesting path to goal (" << goal.pose.position.x
                      << ", " << goal.pose.position.y << ")...\n";

            auto goal_pose_ptr = std::make_shared<geometry_msgs::msg::Pose>(goal.pose);

            std::shared_ptr<nav_msgs::msg::Path> path = navigator_.GetPath(goal_pose_ptr);

            if (!path)
            {
                std::cerr << "No path found (GetPath returned nullptr).\n";
                return 2;
            }

            printPath(*path);
            return 0;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception: " << e.what() << "\n";
            return 1;
        }
    }

private:
    static double yawFromQuat(const geometry_msgs::msg::Quaternion &msg)
    {
        tf2::Quaternion q;
        tf2::fromMsg(msg, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    geometry_msgs::msg::PoseStamped makePose(double x, double y, double z, double yaw_rad, const std::string &frame)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = frame;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        tf2::Quaternion q;

        q.setRPY(0.0, 0.0, yaw_rad);
        pose.pose.orientation = tf2::toMsg(q);
        return pose;
    }

    void setInitialPose(const std::string &frame, double x, double y, double yaw)
    {
        auto init = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

        init->header.stamp = this->now();
        init->header.frame_id = frame;

        auto p = makePose(x, y, 0.0, yaw, frame).pose;
        init->pose.pose = p;

        for (int i = 0; i < 36; i++)
        {
            init->pose.covariance[i] = 0.0;
        }
        init->pose.covariance[0] = 0.25;
        init->pose.covariance[7] = 0.25;
        init->pose.covariance[35] = 0.068;

        auto init_pose_ptr = std::make_shared<geometry_msgs::msg::Pose>(init->pose.pose);
        navigator_.SetInitialPose(init_pose_ptr);
    }

    void printPath(const nav_msgs::msg::Path &path)
    {
        std::cout << "Path has " << path.poses.size()
                  << " poses (frame_id=" << path.header.frame_id << ")\n";
        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            const auto &ps = path.poses[i];
            double yaw = yawFromQuat(ps.pose.orientation);
            std::cout << "  [" << i << "] x=" << ps.pose.position.x
                      << ", y=" << ps.pose.position.y
                      << ", yaw=" << yaw << " rad\n";
        }
    }

    Navigator navigator_;
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<testnavigator>(true);
    int ret = node->run();

    rclcpp::shutdown();
    return ret;
}