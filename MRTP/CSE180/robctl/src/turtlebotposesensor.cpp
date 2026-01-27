#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class PoseSensor : public rclcpp::Node
{
public:
  PoseSensor() : rclcpp::Node("turtlebotposesensor")
  {
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry &msg)
        {
          const auto &p = msg.pose.pose.position;
          RCLCPP_INFO(this->get_logger(), "pose (x=%.3f, y=%.3f, z=%.3f)", p.x, p.y, p.z);
        });
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSensor>());
  rclcpp::shutdown();
  return 0;
}
