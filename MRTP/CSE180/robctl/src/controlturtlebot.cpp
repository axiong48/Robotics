#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("controlturtlebot");
  auto pub  = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  rclcpp::Rate rate(1.0);
  bool straight = true;

  RCLCPP_INFO(node->get_logger(), "Publishing /cmd_vel: alternate straight & rotate @1Hz");

  while (rclcpp::ok()) {
    geometry_msgs::msg::Twist cmd;
    if (straight) {
      cmd.linear.x  = 1.5;   // m/s forward
      cmd.angular.z = 0.0;
    } else {
      cmd.linear.x  = 0.0;
      cmd.angular.z = -0.6;  // rad/s clockwise (negative z = clockwise)
    }
    pub->publish(cmd);
    straight = !straight;
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
