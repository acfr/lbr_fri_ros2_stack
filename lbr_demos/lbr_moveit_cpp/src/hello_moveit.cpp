#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"  // For subscribing to the 7D array (position and orientation)

class MoveGroupNode : public rclcpp::Node {
public:
  MoveGroupNode() : Node("hello_moveit") {
  // Configure node
    auto node_ptr = rclcpp::Node::make_shared("hello_moveit");
    node_ptr->declare_parameter("robot_name", "lbr");
    auto robot_name = node_ptr->get_parameter("robot_name").as_string();

    // Create MoveGroupInterface (lives inside robot_name namespace)
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(
        node_ptr, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description",
                                                                          robot_name));

    // Subscribe to the 'position' topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "position", 10, std::bind(&MoveGroupNode::position_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveGroupNode ready to receive positions.");
  }

private:
  // Callback function to handle position data
  void position_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {

    // Extract position and orientation from the 7D array
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x =  msg->position.x;
    target_pose.position.y = msg->position.y;
    target_pose.position.z = msg->position.z;
    target_pose.orientation.x = msg->orientation.x;
    target_pose.orientation.y = msg->orientation.y;
    target_pose.orientation.z = msg->orientation.z;
    target_pose.orientation.w = msg->orientation.w;

    // Set the target pose
    move_group_interface_->setPoseTarget(target_pose);

    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto error_code = move_group_interface_->plan(plan);

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      // Execute the plan
      move_group_interface_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to the received position.");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create and spin the node
  auto node = std::make_shared<MoveGroupNode>();
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
