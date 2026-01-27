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

static double yawFromQuat(const geometry_msgs::msg::Quaternion &qmsg)
{
  tf2::Quaternion q;
  tf2::fromMsg(qmsg, q);
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::optional<geometry_msgs::msg::Pose> amcl_pose_data;

  auto nodeh = rclcpp::Node::make_shared("trackdifferences");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(nodeh->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  auto subTarget = nodeh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose",
      10,

      [&amcl_pose_data](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
      {
        amcl_pose_data = msg->pose.pose;
      });

  auto subRobot = nodeh->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10,
      // capture tf_buffer, nodeh , and amcl_pose_data 
      [tf_buffer, nodeh, &amcl_pose_data](const nav_msgs::msg::Odometry::SharedPtr msg)
      {

        if (!amcl_pose_data.has_value())
        {
          RCLCPP_INFO_ONCE(nodeh->get_logger(), "Waiting for first /amcl_pose message...");
          return;
        }

        // We have an amcl_pose, so we can proceed.
        // Get a local copy of the pose for this calculation
        geometry_msgs::msg::Pose current_amcl_pose = amcl_pose_data.value();

        try
        {
          // transformation from odom to map
          geometry_msgs::msg::TransformStamped map_to_odom_tf;
          map_to_odom_tf = tf_buffer->lookupTransform(
              "map",            // target_frame
              "odom",           // source_frame
              rclcpp::Time(0)); 

          // 
          geometry_msgs::msg::PoseStamped odom_pose_stamped;
          odom_pose_stamped.header = msg->header;  // Use the header from the odom message
          odom_pose_stamped.pose = msg->pose.pose; // (Corrected from msg-<pose.pose)

          // Transform the odom pose to the map frame
          geometry_msgs::msg::PoseStamped odom_pose_in_map_stamped;
          tf2::doTransform(odom_pose_stamped, odom_pose_in_map_stamped, map_to_odom_tf);

          // 4. Extract the simple 'Pose'
          geometry_msgs::msg::Pose odom_pose_in_map = odom_pose_in_map_stamped.pose;

          // 5. Calculate differences
          double diff_x = current_amcl_pose.position.x - odom_pose_in_map.position.x;
          double diff_y = current_amcl_pose.position.y - odom_pose_in_map.position.y;

          double amcl_yaw = yawFromQuat(current_amcl_pose.orientation);
          double odom_in_map_yaw = yawFromQuat(odom_pose_in_map.orientation);

          double diff_yaw = amcl_yaw - odom_in_map_yaw;
          diff_yaw = wrapToPi(diff_yaw); // Use your helper function

          // 6. Print to screen
          RCLCPP_INFO(nodeh->get_logger(), "Pose Differences (AMCL - Odom in Map):");
          RCLCPP_INFO(nodeh->get_logger(), "  Diff X:   %f", diff_x);
          RCLCPP_INFO(nodeh->get_logger(), "  Diff Y:   %f", diff_y);
          RCLCPP_INFO(nodeh->get_logger(), "  Diff Yaw: %f", diff_yaw);
          RCLCPP_INFO(nodeh->get_logger(), "-------------------------------------");
        }
        catch (const tf2::TransformException &ex)
        {
          // (Completed the try-catch block)
          RCLCPP_WARN(nodeh->get_logger(), "Could not transform 'odom' to 'map': %s", ex.what());
        }
      });

  // Spin the node to process callbacks (This was missing)
  RCLCPP_INFO(nodeh->get_logger(), "trackdifferences node started. Waiting for poses...");
  rclcpp::spin(nodeh);

  rclcpp::shutdown();
  return 0;
}