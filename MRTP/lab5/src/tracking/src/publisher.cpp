#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class trackdifference : public rclcpp::Node
{
public:
    trackdifference() : rclcpp::Node("trackingdiff")
    {
        auto nodeh = rclcpp::Node::make_shared("trackingdiff");

        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(nodeh->get_clock());
        auto tf_listener = std::make_shared<tf_ros::TransformListener>(*tf_buffer);

        auto subTarget = nodeh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, [&amcl_pose_data]
        { const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg }
        {
            amcl_pose_data = msg->pose.pose;
        });

        auto subRobot = nodeh->create_subscription<nav_msgs::Odometry("odom", 10, [&amcl_pose_data, tf_buffer, nodeh]{
            (const nav_msgs::msg::Odometry::SharedPtr msg)

            if (amcl_pose_data.has_value()) {
                RLCPP_INFO_ONCE(nodeh->get_logger(), "waiting till spawn");
                return;
            }

            geometry_msgs::msg::Pose current_amcl_pose = amcl_pose_data.value();

            try {
                
            }
        })
    }

private:
    static double yawQuat(const geometry_msgs::msg::Quaternion &qmsg)
    {
        tf2::Quaternion q;
        tf2::fromMsg(&qmsg, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    static double wrapToPi(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a < -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

}

int
main(
    init::rclcpp(argc, argv);)