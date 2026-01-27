#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RemapNode : public rclcpp::Node {
public:
  RemapNode() : rclcpp::Node("remap") {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "turtle1/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg){
        pub_->publish(*msg);
      });
    RCLCPP_INFO(get_logger(), "Bridging 'turtle1/cmd_vel' -> '/cmd_vel'");
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemapNode>());
  rclcpp::shutdown();
  return 0;
}
