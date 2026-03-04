#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <cmath>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

enum HunterState { HUNTING, LOGGING, CLEARING_ZONE, DONE };

class HunterNode : public rclcpp::Node {
public:
    HunterNode() : Node("hunter_node"), state_(HUNTING), tx_(0.0), ty_(0.0), tz_(0.0), current_yaw_(0.0), actively_hunting_(false), is_enabled_(false) {
        
        target_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/lantern/target_data", 10, std::bind(&HunterNode::autoSteerCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&HunterNode::odomCallback, this, std::placeholders::_1));
            
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/command/trajectory", 10);
        
        start_time_ = this->now();
        timer_ = this->create_wall_timer(100ms, std::bind(&HunterNode::publishLoop, this));

        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_hunting", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && !is_enabled_) {
                    RCLCPP_INFO(this->get_logger(), "HUNTING ACTIVATED by State Machine!");
                    is_enabled_ = true;
                    actively_hunting_ = false; // Reset pursuit flags
                    state_ = HUNTING;
                } else if (!msg->data && is_enabled_) {
                    RCLCPP_INFO(this->get_logger(), "HUNTING DEACTIVATED.");
                    is_enabled_ = false;
                }
            });
            
         done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/hunting_done", 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Track the drone's position and exact Yaw until we actively start hunting
        if (!actively_hunting_) {
            tx_ = msg->pose.pose.position.x;
            ty_ = msg->pose.pose.position.y;
            tz_ = msg->pose.pose.position.z;

            auto q = msg->pose.pose.orientation;
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
        }
    }

    void autoSteerCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        if (!is_enabled_) return; // ADDED: Ignore camera if not enabled
        if ((this->now() - start_time_).seconds() < 2.0) return;
        if (state_ != HUNTING || msg->z == 2.0) return; // Skip if bright spot or not hunting

        actively_hunting_ = true;

        double stop_threshold = 800000.0;
        double centering_gain = 0.0005; 
        current_yaw_ -= msg->x * centering_gain;

        if (msg->y < stop_threshold) {
            // Synced speeds to match manual_pilot.cpp
            double hunt_speed = (msg->y > 100000.0) ? 0.1 : 0.12;
            tx_ += std::cos(current_yaw_) * hunt_speed;
            ty_ += std::sin(current_yaw_) * hunt_speed;

            // Dive condition
            if (msg->y > 30000.0) {
                tz_ -= msg->z * 0.002;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Lantern Reached! LOGGING mode active.");
            state_ = LOGGING;
            reach_time_ = this->now();
            clearing_target_z_ = tz_ + 10.0;
        }
    }

    void publishLoop() {
        if (!is_enabled_) return;
        
        if (state_ == LOGGING) {
            if ((this->now() - reach_time_).seconds() >= 2.0) {
                RCLCPP_INFO(this->get_logger(), "Logging complete. CLEARING ZONE...");
                state_ = CLEARING_ZONE;
                reach_time_ = this->now();
            }
        } else if (state_ == CLEARING_ZONE) {
            if (tz_ < (clearing_target_z_ - 0.1)) {
                tz_ += 0.2; // Vertical Climb
                reach_time_ = this->now();
            } else {
                tx_ += std::cos(current_yaw_) * 0.05; // Forward Burst
                ty_ += std::sin(current_yaw_) * 0.05;
                if ((this->now() - reach_time_).seconds() >= 10.0) {
                    if (state_ != DONE) {
                        RCLCPP_INFO(this->get_logger(), "Zone cleared. DONE.");
                        state_ = DONE;
                        
                        std_msgs::msg::Bool done_msg;
                        done_msg.data = true;
                        done_pub_->publish(done_msg);
                    }
                }
            }
        }
        sendCommand();
    }

    void sendCommand() {
        auto msg = trajectory_msgs::msg::MultiDOFJointTrajectory();
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        geometry_msgs::msg::Transform trans;
        
        trans.translation.x = tx_;
        trans.translation.y = ty_;
        trans.translation.z = tz_;
        
        trans.rotation.z = std::sin(current_yaw_ / 2.0);
        trans.rotation.w = std::cos(current_yaw_ / 2.0);

        point.transforms.push_back(trans);
        point.velocities.push_back(geometry_msgs::msg::Twist()); // Empty twists
        point.accelerations.push_back(geometry_msgs::msg::Twist());
        
        // Match the 0.1s execution time from manual_pilot.cpp
        point.time_from_start.nanosec = 100000000; 

        msg.points.push_back(point);
        traj_pub_->publish(msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    HunterState state_;
    double tx_, ty_, tz_, current_yaw_, clearing_target_z_;
    bool actively_hunting_;
    rclcpp::Time start_time_, reach_time_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
    bool is_enabled_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HunterNode>());
    rclcpp::shutdown();
    return 0;
}
