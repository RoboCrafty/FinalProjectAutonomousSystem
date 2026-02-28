#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class MissionControlNode : public rclcpp::Node {
public:
    MissionControlNode() : Node("mission_control_node"), phase_(0) {
        
        // Publisher for Phase 1: The Path for the Planner
        rclcpp::QoS qos(10);
        qos.transient_local(); 
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", qos);
        
        // Publisher for Phase 0: The Jumpstart for the Controller
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/command/trajectory", 10);
        
        RCLCPP_INFO(this->get_logger(), "Mission Control Initialized. Preparing for Sequence...");

        // State Machine Timer (Ticks every 1 second)
        mission_timer_ = this->create_wall_timer(
            1s, std::bind(&MissionControlNode::stateMachineTick, this));
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
    int phase_;

    void stateMachineTick() {
        if (phase_ == 0) {
            // --- PHASE 0: WAKE UP AND HOVER ---
            RCLCPP_INFO(this->get_logger(), "PHASE 0: Arming Controller & Hovering at Z=10.0...");
            
            trajectory_msgs::msg::MultiDOFJointTrajectory hover_msg;
            hover_msg.header.stamp = this->now();
            hover_msg.header.frame_id = "world";
            hover_msg.joint_names = {"base_link"};

            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
            geometry_msgs::msg::Transform transform;
            
            // Your exact jumpstart coordinates
            transform.translation.x = -36.0;
            transform.translation.y = 10.0;
            transform.translation.z = 10.0;
            
            transform.rotation.x = 0.0;
            transform.rotation.y = 0.0;
            transform.rotation.z = -1.0;
            transform.rotation.w = 0.0;

            point.transforms.push_back(transform);
            
            geometry_msgs::msg::Twist zero_twist;
            point.velocities.push_back(zero_twist);
            point.accelerations.push_back(zero_twist);
            
            hover_msg.points.push_back(point);
            traj_pub_->publish(hover_msg);
            
            phase_ = 1; 
        } 
        else if (phase_ >= 1 && phase_ < 5) {
            // --- WAIT PHASE ---
            RCLCPP_INFO(this->get_logger(), "Waiting for physics to stabilize... (%d/4)", phase_);
            phase_++;
        }
        else if (phase_ == 5) {
            // --- PHASE 1: FLY TO CAVE ---
            RCLCPP_INFO(this->get_logger(), "PHASE 1: Executing Cave Path!");

            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "world";
            path_msg.header.stamp = this->now();

            std::vector<std::vector<double>> raw_points = {
                {-36.0, 10.0, 15.0}, {-60.0, 10.2, 15.0}, {-85.0, 10.1, 15.1}, 
                {-110.0, 10.0, 15.2}, {-135.0, 9.8, 15.2}, {-160.0, 9.7, 15.3},
                {-185.0, 9.6, 15.4}, {-205.0, 9.5, 15.5}, {-225.0, 9.3, 15.5}, 
                {-245.0, 9.2, 15.6}, {-265.0, 9.0, 15.7}, {-285.0, 8.9, 15.6}, 
                {-300.0, 8.7, 15.4}, {-310.0, 8.6, 15.2}, {-312.0, 8.5, 15.1},
                {-315.14, 8.38, 14.99} // Cave Entrance
            };

            for (const auto& pt : raw_points) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = pt[0];
                pose.pose.position.y = pt[1];
                pose.pose.position.z = pt[2];
                path_msg.poses.push_back(pose);
            }

            path_pub_->publish(path_msg);
            
            phase_ = 6; 
            mission_timer_->cancel(); // Stop the timer so it doesn't repeat
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlNode>());
    rclcpp::shutdown();
    return 0;
}
