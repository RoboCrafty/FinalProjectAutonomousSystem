#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <chrono>

class MissionControlNode : public rclcpp::Node {
public:
    MissionControlNode() : Node("mission_control_node") {
        
        // Publisher to send the path to the trajectory generator
        // Using "transient_local" durability ensures the message isn't lost if sent too fast
        rclcpp::QoS qos(10);
        qos.transient_local(); 
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", qos);
        
        RCLCPP_INFO(this->get_logger(), "Mission Control Initialized.");

        // We use a tiny 0.5s delay to guarantee the publisher has registered on the ROS network
        // before firing off the waypoints.
        startup_delay_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MissionControlNode::executeMission, this));
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr startup_delay_;

    void executeMission() {
        // Immediately cancel this timer so it only runs exactly once
        startup_delay_->cancel();

        RCLCPP_INFO(this->get_logger(), "Executing Phase 1: Flying to Cave Entrance!");

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "world";
        path_msg.header.stamp = this->now();

        // The hardcoded waypoints to the cave entrance
        std::vector<std::vector<double>> raw_points = {
            {-36.0, 10.0, 15.0}, {-60.0, 10.2, 15.0}, {-85.0, 10.1, 15.1}, 
            {-110.0, 10.0, 15.2}, {-135.0, 9.8, 15.2}, {-160.0, 9.7, 15.3},
            {-185.0, 9.6, 15.4}, {-205.0, 9.5, 15.5}, {-225.0, 9.3, 15.5}, 
            {-245.0, 9.2, 15.6}, {-265.0, 9.0, 15.7}, {-285.0, 8.9, 15.6}, 
            {-300.0, 8.7, 15.4}, {-310.0, 8.6, 15.2}, {-312.0, 8.5, 15.1}, 
            {-315.14, 8.38, 14.99} // Cave Entrance
        };

        // Convert double arrays to PoseStamped messages
        for (const auto& pt : raw_points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = pt[0];
            pose.pose.position.y = pt[1];
            pose.pose.position.z = pt[2];
            path_msg.poses.push_back(pose);
        }

        // Publish the path to your Trajectory Generator
        path_pub_->publish(path_msg);
        
        RCLCPP_INFO(this->get_logger(), "Waypoints dispatched to Trajectory Generator.");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlNode>());
    rclcpp::shutdown();
    return 0;
}

