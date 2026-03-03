/*
 * Simple example that shows a trajectory planner using
 * mav_trajectory_generation in ROS2.
 *
 * Build + run via your ROS2 launch setup, e.g.
 *   ros2 run basic_waypoint_pkg basic_waypoint_node
 */

 #include <memory>
 #include <iostream>
 
 #include "rclcpp/rclcpp.hpp"
 #include "Eigen/Dense"
 
 #include "basic_waypoint_pkg/planner.hpp"
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = rclcpp::Node::make_shared("simple_planner");

  // Instantiate the planner. 
  // The constructor sets up the publishers and subscribers.
  BasicPlanner planner(node);

  RCLCPP_INFO(node->get_logger(), "Planner Node started. Waiting for goal_pose...");

  // Spin the node to keep it alive and processing callbacks.
  // This replaces the manual 'spin_some' loop and the hardcoded planning.
  rclcpp::spin(node);

  // Shutdown once the node is killed (e.g., Ctrl+C)
  rclcpp::shutdown();
  return 0;
}
 