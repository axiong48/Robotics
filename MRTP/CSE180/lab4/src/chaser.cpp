
#include <cmath>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using turtlesim::msg::Pose;
using geometry_msgs::msg::Twist;

static double normalize_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class ChaserNode : public rclcpp::Node {
public:
  ChaserNode() : rclcpp::Node("chaser") {
    k_lin_ = declare_parameter("k_lin", 1.2);
    k_ang_ = declare_parameter("k_ang", 4.0);
    vmax_  = declare_parameter("vmax",  2.0);
    wmax_  = declare_parameter("wmax",  6.0);
    stop_dist_ = declare_parameter("stop_dist", 0.05);

    pub_cmd_ = create_publisher<Twist>("/T1/cmd_vel", 10);

    sub_self_ = create_subscription<Pose>("/T1/pose", 10,
      [this](const Pose & msg){ self_pose_ = msg; });
    sub_target_ = create_subscription<Pose>("/turtle1/pose", 10,
      [this](const Pose & msg){ target_pose_ = msg; });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ChaserNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "Chaser node up: T1 will chase turtle1");
  }

private:
  void control_loop() {
    if (!self_pose_ || !target_pose_) {
      return;
    }

    const auto & s = *self_pose_;
    const auto & t = *target_pose_;

    const double dx = t.x - s.x;
    const double dy = t.y - s.y;
    const double dist = std::hypot(dx, dy);

    Twist cmd;

    if (dist < stop_dist_) {
      // Close enough; stop
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      pub_cmd_->publish(cmd);
      return;
    }

    // Desired bearing and heading error
    const double bearing = std::atan2(dy, dx);
    double heading_err = normalize_angle(bearing - s.theta);

    // Proportional control
    double v = k_lin_ * dist;
    double w = k_ang_ * heading_err;

    // Gate linear speed by how well aligned we are (helps avoid sideways drift)
    const double align = std::cos(heading_err);
    v *= std::max(0.0, align);

    // Saturate
    if (v > vmax_) v = vmax_;
    if (v < -vmax_) v = -vmax_;
    if (w > wmax_) w = wmax_;
    if (w < -wmax_) w = -wmax_;

    cmd.linear.x = v;
    cmd.angular.z = w;
    pub_cmd_->publish(cmd);
  }

  // Params
  double k_lin_, k_ang_, vmax_, wmax_, stop_dist_;

  // I/O
  rclcpp::Publisher<Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<Pose>::SharedPtr sub_self_, sub_target_;

  // State
  std::optional<Pose> self_pose_, target_pose_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChaserNode>());
  rclcpp::shutdown();
  return 0;
}
