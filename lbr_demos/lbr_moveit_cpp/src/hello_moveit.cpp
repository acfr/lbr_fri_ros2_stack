#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "lbr_interfaces/srv/pose_service.hpp" 

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
    // Create the service that handles pose requests
    service_ = this->create_service<lbr_interfaces::srv::PoseService>(
        "pose_service", std::bind(&MoveGroupNode::pose_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "MoveGroupNode ready to receive positions.");
  }

private:
  // Callback function to handle position data
  void pose_service_callback(const std::shared_ptr<lbr_interfaces::srv::PoseService::Request> request,
                             std::shared_ptr<lbr_interfaces::srv::PoseService::Response> response)
  {


    // Set a target Pose
    auto const target_pose = [request]{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = request->target_pose.orientation.w;
      msg.orientation.x = request->target_pose.orientation.x;
      msg.orientation.y = request->target_pose.orientation.y;
      msg.orientation.z = request->target_pose.orientation.z;
      msg.position.x = request->target_pose.position.x;
      msg.position.y = request->target_pose.position.y;
      msg.position.z = request->target_pose.position.z;
      return msg;
    }();

    move_group_interface_->setPoseTarget(target_pose);
    // Set the target pose
    RCLCPP_INFO(this->get_logger(), "Recieved a request %f", target_pose.position.x);
    RCLCPP_INFO(this->get_logger(), "Recieved a request %f", target_pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Recieved a request %f", target_pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Recieved a request %f", target_pose.orientation.x);
    RCLCPP_INFO(this->get_logger(), "Recieved a request %f", target_pose.orientation.y);
    RCLCPP_INFO(this->get_logger(), "Recieved a request %f", target_pose.orientation.z);
    RCLCPP_INFO(this->get_logger(), "Recieved a request %f", target_pose.orientation.w);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto error_code = move_group_interface_->plan(plan);

    // RCLCPP_INFO(this->get_logger(), "Recieved a request %f", request->target_pose.pose.position.x);
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      // Execute the plan
      auto execution_result = move_group_interface_->execute(plan);
      if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Movement executed successfully.");
        response->success = true;  // Set the response to true
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute the movement.");
        response->success = false;  // Set the response to false
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to the requested pose.");
      response->success = false;  // Set the response to false
    }
  }

  rclcpp::Service<lbr_interfaces::srv::PoseService>::SharedPtr service_;
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