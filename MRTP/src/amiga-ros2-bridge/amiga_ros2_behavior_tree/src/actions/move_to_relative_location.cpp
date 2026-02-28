#include "amiga_ros2_behavior_tree/actions/move_to_relative_location.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

namespace amiga_bt {

MoveToRelativeLocation::MoveToRelativeLocation(const std::string &name,
                                               const BT::NodeConfig &config,
                                               const BT::RosNodeParams &params)
    : BT::RosActionNode<MoveInFrame>(name, config, params) {}

BT::PortsList MoveToRelativeLocation::providedPorts() {
  return providedBasicPorts(
      {BT::InputPort<double>("x"), BT::InputPort<double>("y")});
}

bool MoveToRelativeLocation::setGoal(Goal &goal) {
  double x = 0.0, y = 0.0;

  if (getInput("x", x) && getInput("y", y)) {
    RCLCPP_DEBUG(logger(), "Received x/y input. Making relative movement.");
  } else {
    RCLCPP_ERROR(logger(), "Missing x/y input");
    return false;
  }

  // Set the goal pose
  goal.x = x;
  goal.y = y;

  RCLCPP_INFO(logger(),
              "Moving to relative location: (x=%.2f, y=%.2f)",
              x, y);

  return true;
}

BT::NodeStatus MoveToRelativeLocation::onResultReceived(
    const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Navigation finished with code: %d", int(result.code));

  // Check if the action succeeded
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Navigation succeeded!");
    return BT::NodeStatus::SUCCESS;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger(), "Navigation was canceled");
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(logger(), "Navigation failed or was aborted");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveToRelativeLocation::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(),
              "Distance remaining: %.2f m",
              feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
