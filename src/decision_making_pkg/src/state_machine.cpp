#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cmath>

using namespace std::chrono_literals;

enum class MissionState {
    APPROACH_CAVE,      // Hover, Approach, and Enter
    EXPLORE_CAVE,       // Wait for map and start frontier exploration
    MISSION_COMPLETED,
};

class StateMachineNode : public rclcpp::Node {
public:
    StateMachineNode() : Node("state_machine_node"), current_state_(MissionState::APPROACH_CAVE), sub_step_(0), has_odom_(false), target_sent_(false) {

        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        explore_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_exploration", 10);
        octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("/octomap_server/reset");
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_ = msg->pose.pose.position;
                has_odom_ = true;
            });

        explore_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/exploration_complete", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) exploration_finished_received_ = true;
            });

        mission_timer_ = this->create_wall_timer(500ms, std::bind(&StateMachineNode::stateMachineTick, this));
        RCLCPP_INFO(this->get_logger(), "State Machine Ready");
    }

private:
    void stateMachineTick() {
        if (!has_odom_) return;

        switch (current_state_) {
            case MissionState::APPROACH_CAVE:
                handleApproachPhase();
                break;

            case MissionState::EXPLORE_CAVE:
                handleExplorationPhase();
                break;

            case MissionState::MISSION_COMPLETED:
                setExploration(false);
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission completed. Hovering at start.");
                break;
        }
    }



    //APPROACH STATE
    void handleApproachPhase() {
        setExploration(false); // Ensure explorer is OFF

        // Climb to Hover point
        if (sub_step_ == 0) {
            if (!target_sent_) { sendTarget(-36.0, 10.0, 25.0, 0.0, 0.0, 1.0, 0.0); target_sent_ = true; RCLCPP_INFO(this->get_logger(), "Climbing..."); }
            if (isReached(-36.0, 10.0, 25.0)) { sub_step_++; target_sent_ = false; }
        }
        //Approach Entrance
        else if (sub_step_ == 1) {
            if (!target_sent_) { sendTarget(-310.14, 8.38, 15.0); target_sent_ = true; RCLCPP_INFO(this->get_logger(), "Approaching Entrance..."); }
            if (isReached(-310.14, 8.38, 15.0)) { 
                resetOctomap(); // Clear old map data before entering
                sub_step_++; target_sent_ = false; 
            }
        }
        //Enter Cave
        else if (sub_step_ == 2) {
            if (!target_sent_) { sendTarget(-330.14, 8.38, 15.0); target_sent_ = true; RCLCPP_INFO(this->get_logger(), "Entering Cave..."); }
            if (isReached(-330.14, 8.38, 15.0)) {
                start_wait_time_ = this->now();
                current_state_ = MissionState::EXPLORE_CAVE; // Move to next major Phase
                sub_step_ = 0; // Reset sub_step for next phase if needed
                target_sent_ = false;
            }
        }
    }



    //EXPLORATION STATE
    void handleExplorationPhase() {
        //Wait for map sensors to fill after the reset
        if ((this->now() - start_wait_time_) < 4s) {
            setExploration(false);
            return;
        }

        // Once sensors are ready, start the actual explorer
        if (exploration_finished_received_) {
            current_state_ = MissionState::MISSION_COMPLETED;
        } else {
            setExploration(true);
            RCLCPP_INFO_ONCE(this->get_logger(), "Frontier Explorer active.");
        }
    }



    // HELPER FUNCTIONS
    void setExploration(bool enable) {
        std_msgs::msg::Bool msg;
        msg.data = enable;
        explore_pub_->publish(msg);
    }


    void resetOctomap() {
        if (octomap_reset_client_->wait_for_service(1s)) {
            octomap_reset_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
            RCLCPP_INFO(this->get_logger(), "Octomap reset.");
        }
    }


    void sendTarget(double x, double y, double z, double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 0.0) {
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
        double dist = std::sqrt(std::pow(tx - current_pos_.x, 2) + std::pow(ty - current_pos_.y, 2) + std::pow(tz - current_pos_.z, 2));
        return dist < 1.5;
    }

    // Members
    MissionState current_state_;
    int sub_step_;
    bool has_odom_;
    bool target_sent_;
    bool exploration_finished_received_ = false;

    geometry_msgs::msg::Point current_pos_;
    rclcpp::Time start_wait_time_;
    
    // Pubs/Subs
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_finished_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();
    return 0;
}