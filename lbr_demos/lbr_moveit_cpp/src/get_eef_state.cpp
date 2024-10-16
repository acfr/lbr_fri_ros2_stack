#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <geometry_msgs/msg/pose.hpp>

// Combined RobotMover class, which includes service and pose printing functionality
class RobotMover : public rclcpp::Node {
public:
  // Constructor to initialize the node, MoveGroupInterface, and executor
  RobotMover(const rclcpp::NodeOptions &options)
  : rclcpp::Node("robot_control", options), // Initialize the node with the name "robot_control"
    node_(std::make_shared<rclcpp::Node>("move_group_interface")), // Create an additional ROS node
    move_group_interface_(node_, "arm"), // Initialize MoveGroupInterface for controlling the arm
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) // Create a single-threaded executor
  {
    // Create the service for printing the current pose
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "print_current_pose", 
      std::bind(&RobotMover::handlePrintRequest, this, std::placeholders::_1, std::placeholders::_2)
    );

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("eef_pose", 10);

    // Create a timer to publish the end-effector pose at 10 Hz
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),  // 100ms = 10 Hz
                                   std::bind(&RobotMover::publish_pose, this));


    // Add the node to the executor and start the executor thread
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(node_->get_logger(), "Starting executor thread"); // Log message indicating the thread start
      executor_->spin(); // Run the executor to process callbacks
    });
  }

  // Function to print the current end-effector pose
  void printCurrentPose() {
    auto current_pose = move_group_interface_.getCurrentPose().pose; // Get the current pose
    std::cout << "Current Pose:" << std::endl;
    std::cout << "Position: (" << current_pose.position.x << ", "
              << current_pose.position.y << ", "
              << current_pose.position.z << ")" << std::endl;
    std::cout << "Orientation: (" << current_pose.orientation.x << ", "
              << current_pose.orientation.y << ", "
              << current_pose.orientation.z << ", "
              << current_pose.orientation.w << ")" << std::endl;
  }

  void publish_pose()
  {
    // Publish the current pose of the end effector to the topic
    auto current_pose = move_group_interface_.getCurrentPose().pose; // Get the current pose
    pose_publisher_->publish(current_pose);
  }


private:
  // Service callback function to handle pose printing requests
  void handlePrintRequest(const std_srvs::srv::Trigger::Request::SharedPtr,
                            std_srvs::srv::Trigger::Response::SharedPtr response) {
    // Print the current pose in the service callback
    RCLCPP_INFO(node_->get_logger(), "Service Callback: About to call printCurrentPose in Private");
    printCurrentPose(); // Print the robot's current pose

    // Set the response to indicate success
    response->success = true;
  }

  // Member variables
  rclcpp::Node::SharedPtr node_; // Additional ROS node pointer
  moveit::planning_interface::MoveGroupInterface move_group_interface_;  // MoveIt interface for controlling the arm
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;  // Service pointer for pose requests
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
  std::thread executor_thread_;  // Thread to run the executor
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

// Main function - Entry point of the program
int main(int argc, char** argv) {
  rclcpp::init(argc, argv); // Initialize ROS 2

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true); // Allow automatic parameter declaration
  node_options.use_intra_process_comms(false); // Disable intra-process communication

  auto node = std::make_shared<RobotMover>(node_options); // Create the RobotMover object and start the node

  rclcpp::spin(node); // Spin the main thread to process callbacks

  rclcpp::shutdown(); // Shutdown the ROS 2 system
  return 0; // Exit the program
}