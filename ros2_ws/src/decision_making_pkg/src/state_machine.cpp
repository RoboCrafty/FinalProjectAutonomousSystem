#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/vector3.hpp> // Required for lantern data
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

enum class MissionState {
    WAYPOINT_HOVER,
    APPROACH_CAVE,
    EXPLORE_CAVE,
    STABILIZING,
    HUNTING_LANTERN,    // Added hunting phase
    MISSION_COMPLETED,
};

class StateMachineNode : public rclcpp::Node {
public:
    StateMachineNode() : Node("state_machine_node"), current_state_(MissionState::WAYPOINT_HOVER), has_odom_(false), target_sent_(false), lanterns_found_(0){
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        
        // Publisher to wake up the Python Explorer
        explore_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_exploration", 10);
        
        // Subscription to listen for lanterns from the perception tracker
        lantern_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/lantern/target_data", 10, std::bind(&StateMachineNode::lanternCallback, this, std::placeholders::_1));

        // Client to reset the Octomap
        octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("/octomap_server/reset");
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/current_state_est", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_pos_ = msg->pose.pose.position;
            
            // Convert Quaternion to Euler angles to get Yaw
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, current_yaw_); // Store current heading
            
            has_odom_ = true;
        });

        mission_timer_ = this->create_wall_timer(500ms, std::bind(&StateMachineNode::stateMachineTick, this));
        RCLCPP_INFO(this->get_logger(), "State Machine Ready");
        
        hunt_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_hunting", 10);
        
        // ADDED: Subscriber to listen for when the Hunter is finished
        hunt_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hunting_done", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && current_state_ == MissionState::HUNTING_LANTERN) {
                    bool is_actually_a_duplicate = false;
                    for (const auto& saved_pos : secured_lanterns_) {
                        double d = std::sqrt(std::pow(current_pos_.x - saved_pos.x, 2) + 
                                             std::pow(current_pos_.y - saved_pos.y, 2));
                        if (d < 50.0) { is_actually_a_duplicate = true; break; }
                    }

                    if (is_actually_a_duplicate) {
                        RCLCPP_WARN(this->get_logger(), "POST-HUNT GUARD: Duplicate ignored at finish line.");
                        current_state_ = MissionState::EXPLORE_CAVE;
                        explore_pub_->publish(std_msgs::msg::Bool().set__data(true));
                        return;
                    }
                    secured_lanterns_.push_back(current_pos_);
                    lanterns_found_++;
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "SUCCESS: Secured Lantern #%d", lanterns_found_);
                    RCLCPP_INFO(this->get_logger(), 
                        "LANTERN POSITION -> X: %.2f, Y: %.2f, Z: %.2f", 
                        current_pos_.x, current_pos_.y, current_pos_.z);
                    
                    // Turn off the hunter immediately
                    std_msgs::msg::Bool toggle_msg;
                    toggle_msg.data = false;
                    hunt_pub_->publish(toggle_msg);

                    if (lanterns_found_ >= 4) {
                        RCLCPP_INFO(this->get_logger(), "Lantern %d/4 secured! ALL LANTERNS FOUND. Mission Complete.", lanterns_found_);
                        current_state_ = MissionState::MISSION_COMPLETED;
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Lantern %d/4 secured! Resuming exploration...", lanterns_found_);
                        rclcpp::sleep_for(2s);
                        
                        current_state_ = MissionState::EXPLORE_CAVE;
                        
                        // Wake the explorer back up immediately
                        toggle_msg.data = true;
                        explore_pub_->publish(toggle_msg);
                    }
                }
            });
    }

private:
    // Callback to trigger the switch from exploration to hunting
    void lanternCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        if (current_state_ == MissionState::EXPLORE_CAVE && msg->y > 5000.0) {
            
            // 1. Project the estimated lantern position in the world
            // At area 5000, the drone is usually 2.5m away.
            double estimated_dist = 2.5; 
            double projected_x = current_pos_.x + (estimated_dist * std::cos(current_yaw_));
            double projected_y = current_pos_.y + (estimated_dist * std::sin(current_yaw_));

            // 2. High-Confidence Guard (50 meters)
            // If this projected point is near a logged lantern, it's a duplicate.
            bool is_duplicate = false;
            const double guard_radius = 50.0; 

            for (const auto& past_pos : secured_lanterns_) {
                double dist = std::sqrt(std::pow(projected_x - past_pos.x, 2) + 
                                        std::pow(projected_y - past_pos.y, 2));
                if (dist < guard_radius) {
                    is_duplicate = true;
                    break;
                }
            }

            if (is_duplicate) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                    "GUARD: Spotted an old lantern within 50m. Ignoring.");
                return; 
            }

            // 3. New Lantern Confirmed -> Start Anti-Jerk Handover
            RCLCPP_INFO(this->get_logger(), "NEW LANTERN VALIDATED! Area: %.2f. Braking...", msg->y);
            
            // Disable explorer immediately to stop momentum
            auto toggle = std_msgs::msg::Bool();
            toggle.data = false;
            explore_pub_->publish(toggle);

            // Optional: Active Braking - command drone to hover at current spot
            sendTarget(current_pos_.x, current_pos_.y, current_pos_.z);

            current_state_ = MissionState::STABILIZING; 

            auto buffer_timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
            *buffer_timer = this->create_wall_timer(2s, [this, buffer_timer]() {
                RCLCPP_INFO(this->get_logger(), "Drone stable. Starting Hunt.");
                current_state_ = MissionState::HUNTING_LANTERN; 
                (*buffer_timer)->cancel(); 
            });
        }
    }

    void stateMachineTick() {
        if (!has_odom_) return;
        
        // 1. Record Breadcrumbs every 5 meters
        if (breadcrumbs_.empty() || isReached(breadcrumbs_.back().x, breadcrumbs_.back().y, breadcrumbs_.back().z) == false) {
            if (breadcrumbs_.empty() || std::sqrt(std::pow(current_pos_.x - breadcrumbs_.back().x, 2) + 
                std::pow(current_pos_.y - breadcrumbs_.back().y, 2)) > breadcrumb_spacing_) {
                breadcrumbs_.push_back(current_pos_);
            }
        }

        // 2. Autonomous Loop Detection
        if (current_state_ == MissionState::EXPLORE_CAVE && breadcrumbs_.size() > 10) {
            for (size_t i = 0; i < breadcrumbs_.size() - 10; ++i) { // Skip recent crumbs
                double d = std::sqrt(std::pow(current_pos_.x - breadcrumbs_[i].x, 2) + 
                                    std::pow(current_pos_.y - breadcrumbs_[i].y, 2));
                if (d < loop_closure_radius_) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "LOOP DETECTED! Returning to known junction area.");
                    // This triggers the explorer to prioritize "Unknown" side-paths
                    break; 
                }
            }
        }

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
                    sendTarget(-328.14, 8.38, 15.0); 
                    RCLCPP_INFO(this->get_logger(), "Approaching cave entrance...");
                    target_sent_ = true;
                }
                
                if (isReached(-315.14, 8.38, 15.0)) {
                    // Reset the Octomap before entering the cave
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
                // Continuously publish 'True' to keep the Python node active
                std_msgs::msg::Bool explore_msg;
                explore_msg.data = true;
                explore_pub_->publish(explore_msg);
                RCLCPP_INFO_ONCE(this->get_logger(), "Handing control over to Exploration Node...");
                break;
            }
            
            case MissionState::STABILIZING:
                // Do nothing here! This is our "Brake" period.
                // The drone will just drift/hover while the timer runs.
                break;

            case MissionState::HUNTING_LANTERN:
            {
                // ADDED: Continuously publish 'True' to keep the Hunter active
                std_msgs::msg::Bool hunt_msg;
                hunt_msg.data = true;
                hunt_pub_->publish(hunt_msg);
                RCLCPP_INFO_ONCE(this->get_logger(), "Handing control over to Hunting Node...");
                break;
            }

            case MissionState::MISSION_COMPLETED:
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission completed");
                break;
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
        double dist = std::sqrt(std::pow(tx - current_pos_.x, 2) +
                                std::pow(ty - current_pos_.y, 2) +
                                std::pow(tz - current_pos_.z, 2));
        return dist < 1.5;
    }

    // Members
    MissionState current_state_;
    geometry_msgs::msg::Point current_pos_;
    geometry_msgs::msg::Quaternion current_ori_;
    bool has_odom_, target_sent_;
    int lanterns_found_; // ADDED: Counter for the lanterns
    std::vector<geometry_msgs::msg::Point> breadcrumbs_;
    const double breadcrumb_spacing_ = 5.0; // Meters
    const double loop_closure_radius_ = 4.0; // Meters
    std::vector<geometry_msgs::msg::Point> secured_lanterns_;
    const double lantern_proximity_threshold_ = 10.0;
    double current_yaw_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr lantern_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hunt_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hunt_done_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();
    return 0;
}
