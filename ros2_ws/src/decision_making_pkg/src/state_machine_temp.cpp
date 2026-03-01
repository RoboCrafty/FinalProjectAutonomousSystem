#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cmath>

using namespace std::chrono_literals;

// Matches your exact mission flow
enum class MissionState {
    WAYPOINT_HOVER,
    APPROACH_CAVE,
    EXPLORE_CAVE, 
    MISSION_COMPLETED,
};

class StateMachineNode : public rclcpp::Node {
public:
    StateMachineNode() : Node("state_machine_node"), current_state_(MissionState::WAYPOINT_HOVER), has_odom_(false), target_sent_(false) {
        
        // Publishers
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        explore_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_exploration", 10);
        
        // Octomap Reset Client
        octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("/octomap_server/reset");
        
        // Odometry Subscriber - Direct link to your estimator
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_ = msg->pose.pose.position;
                current_ori_ = msg->pose.pose.orientation;
                has_odom_ = true;
            });

        // 500ms Tick - Matches your NBV timing
        mission_timer_ = this->create_wall_timer(500ms, std::bind(&StateMachineNode::stateMachineTick, this));
        RCLCPP_INFO(this->get_logger(), "State Machine Initialized (ERRT Mode)");
    }

private:
    void stateMachineTick() {
        if (!has_odom_) return;

        switch (current_state_) {
            case MissionState::WAYPOINT_HOVER:
                if (!target_sent_) {
                    // Your exact climb coordinates
                    sendTarget(-36.0, 10.0, 25.0, 0.0, 0.0, 1.0, 0.0);
                    RCLCPP_INFO(this->get_logger(), "Phase 1: Climbing to Hover...");
                    target_sent_ = true;
                }
                
                if (isReached(-36.0, 10.0, 25.0)) {
                    current_state_ = MissionState::APPROACH_CAVE;
                    target_sent_ = false;
                }
                break;

            case MissionState::APPROACH_CAVE:
                if (!target_sent_) {
                    // Your exact entrance coordinates
                    sendTarget(-328.14, 8.38, 15.0); 
                    RCLCPP_INFO(this->get_logger(), "Phase 2: Approaching Cave Mouth...");
                    target_sent_ = true;
                }
                
                // Using 315.14 for the distance check as per your original logic
                if (isReached(-315.14, 8.38, 15.0)) {
                    
                    // CLEAN SLATE: Reset Octomap so ERRT only maps the cave interior
                    if (octomap_reset_client_->wait_for_service(1s)) {
                        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                        octomap_reset_client_->async_send_request(request);
                        RCLCPP_INFO(this->get_logger(), "Octomap Reset! Deleting outside world data.");
                    }
                    
                    current_state_ = MissionState::EXPLORE_CAVE; 
                }
                break;

            case MissionState::EXPLORE_CAVE:
            {
                // WAKE UP ERRT: Continuously publish 'True' to drive the path-finding tree
                std_msgs::msg::Bool explore_msg;
                explore_msg.data = true;
                explore_pub_->publish(explore_msg);
                
                RCLCPP_INFO_ONCE(this->get_logger(), "Phase 3: ERRT Tree Exploration Active.");
                break;
            }

            case MissionState::MISSION_COMPLETED:
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission completed");
                break;
        }
    }

    void sendTarget(double x, double y, double z, double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.x = qx;
        msg.pose.orientation.y = qy;
        msg.pose.orientation.z = qz;
        msg.pose.orientation.w = qw;
        target_pub_->publish(msg);
    }

    bool isReached(double tx, double ty, double tz) {
        double dist = std::sqrt(std::pow(tx - current_pos_.x, 2) +
                                std::pow(ty - current_pos_.y, 2) +
                                std::pow(tz - current_pos_.z, 2));
        return dist < 1.5; // Your standard tolerance
    }

    // Handlers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
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