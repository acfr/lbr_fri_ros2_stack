#ifndef LBR_FRI_ROS2__APP_HPP_
#define LBR_FRI_ROS2__APP_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/thread_priority.hpp"

#include "friClientApplication.h"
#include "friUdpConnection.h"

#include "lbr_fri_ros2/client.hpp"

namespace lbr_fri_ros2 {
/**
 * @brief The App has a node_ptr for exposing FRI methods to services. It shares this node_ptr with
 * the #client_ptr_, which reads commands / write states via realtime safe topics.
 *
 * Services:
 * - <b>open_udp_socket</b> (lbr_fri_msgs::srv::AppConnect)
 * Opens UDP port to FRI. Creates #run_thread_ptr_ thread via #on_app_connect_ that calls #run_ to
 * communicate with the robot.
 * - <b>close_udp_socket</b> (lbr_fri_msgs::srv::AppDisconnect)
 * Closes UDP port to FRI. Finishes #run_thread_ptr_ thread via #on_app_disconnect_ through ending
 * #run_.
 *
 */
class App {
public:
  App(const rclcpp::Node::SharedPtr node_ptr, const std::shared_ptr<Client> client_ptr);
  ~App();

  /**
   * @brief Opens a UDP port and spawns the #run_thread_ptr_.
   *
   * @param[in] port_id The port id, allowed values [30200, 30209]
   * @param[in] remote_host The address of the remote host
   * @return true if successfully opened / already open
   * @return false if failed to open
   *
   * @throws std::range_error if invalid port_id
   *
   */
  bool open_udp_socket(const int &port_id = 30200, const char *const remote_host = NULL);

  /**
   * @brief Closes the UDP port and joins the #run_thread_ptr_.
   *
   * @return true if closed successfully / already closed
   * @return false if failed to close
   *
   * @throws std::runtime_error if #run_thread_ptr_ fails to join
   *
   */
  bool close_udp_socket();

  /**
   * @brief Exchanges commands / states between ROS and the FRI.
   *
   * Calls step() on #app_ptr_, which callbacks #client_ptr_. #client_ptr_ reads commands / write
   * states through realtime safe topics.
   *
   */
  void run(int rt_prio = 80);

  void stop_run();

protected:
  /**
   * @brief Checks for valid port id.
   *
   * @param[in] port_id The port id
   * @return true if port id in [30200, 30209]
   * @return false else
   */
  bool valid_port_(const int &port_id);

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr_;

  std::atomic_bool running_;
  std::unique_ptr<std::thread> run_thread_ptr_; /**< Thread running the #run_ method.*/

  std::shared_ptr<Client> client_ptr_; /**< Writes commands to / reads states from robot.*/
  std::unique_ptr<KUKA::FRI::UdpConnection>
      connection_ptr_; /**< UDP connection for reading states / writing commands.*/
  std::unique_ptr<KUKA::FRI::ClientApplication>
      app_ptr_; /**< FRI client application that callbacks #client_ptr_ methods.*/
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__APP_HPP_
