#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp> // <-- NEW: Added Path message
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cmath>

using namespace std::chrono_literals;

enum class MissionState {
    WAYPOINT_HOVER,
    APPROACH_CAVE,
    EXPLORE_CAVE,        
    MISSION_COMPLETED,
};

class StateMachineNode : public rclcpp::Node {
public:
    StateMachineNode() : Node("state_machine_node"), current_state_(MissionState::WAYPOINT_HOVER), has_odom_(false), target_sent_(false) {
        
        // UPGRADE: Now publishing a Path to /rrt_path instead of a Pose to /next_setpoint
        target_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rrt_path", 10);
        
        explore_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_exploration", 10);
        octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("/octomap_server/reset");
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_ = msg->pose.pose.position;
                current_ori_ = msg->pose.pose.orientation;
                has_odom_ = true;
            });

        mission_timer_ = this->create_wall_timer(500ms, std::bind(&StateMachineNode::stateMachineTick, this));
        RCLCPP_INFO(this->get_logger(), "State Machine Ready. Speaking the new Path language!");
    }

private:
    void stateMachineTick() {
        if (!has_odom_) return;

        switch (current_state_) {
            case MissionState::WAYPOINT_HOVER:
                if (!target_sent_) {
                    sendTarget(-36.0, 10.0, 25.0, 0.0, 0.0, 1.0, 0.0);
                    RCLCPP_INFO(this->get_logger(), "Climbing...");
                    target_sent_ = true;
                }
                
                if (isReached(-36.0, 10.0, 25.0)) {
                    current_state_ = MissionState::APPROACH_CAVE;
                    target_sent_ = false;
                }
                break;

            case MissionState::APPROACH_CAVE:
                if (!target_sent_) {
                    // Using the rotation fix we did earlier so it doesn't fly backwards!
                    sendTarget(-330.0, 6.38, 15.0, 0.0, 0.0, 1.0, 0.0); 
                    RCLCPP_INFO(this->get_logger(), "Approaching cave entrance...");
                    target_sent_ = true;
                }
                
                if (isReached(-330.00, 6.38, 15.0)) {
                    if (octomap_reset_client_->wait_for_service(1s)) {
                        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                        octomap_reset_client_->async_send_request(request);
                        RCLCPP_INFO(this->get_logger(), "Octomap reset triggered. Deleting outside world!");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Octomap reset service not available!");
                    }
                    current_state_ = MissionState::EXPLORE_CAVE; 
                }
                break;

            case MissionState::EXPLORE_CAVE:
            {
                std_msgs::msg::Bool explore_msg;
                explore_msg.data = true;
                explore_pub_->publish(explore_msg);
                RCLCPP_INFO_ONCE(this->get_logger(), "Handing control over to Exploration Node...");
                break;
            }

            case MissionState::MISSION_COMPLETED:
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission completed");
                break;
        }
    }

    // UPGRADE: Wraps the single coordinate into a Path message
    void sendTarget(double x, double y, double z, double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "world";

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;

        // Push the single waypoint into the path
        path_msg.poses.push_back(pose);
        
        target_pub_->publish(path_msg);
    }

    bool isReached(double tx, double ty, double tz) {
        double dist = std::sqrt(std::pow(tx - current_pos_.x, 2) +
                                std::pow(ty - current_pos_.y, 2) +
                                std::pow(tz - current_pos_.z, 2));
        return dist < 1.5;
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_pub_; // <-- Changed to Path
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_pub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
    geometry_msgs::msg::Point current_pos_;
    geometry_msgs::msg::Quaternion current_ori_;
    bool has_odom_, target_sent_;
    MissionState current_state_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();
    return 0;
}