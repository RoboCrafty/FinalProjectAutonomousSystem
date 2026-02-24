#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"

using namespace std::chrono_literals;

class ManualPilot : public rclcpp::Node {
public:
    ManualPilot() : Node("manual_pilot") {
        target_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/lantern/target_data", 10, std::bind(&ManualPilot::auto_steer_callback, this, std::placeholders::_1));

        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "/command/trajectory", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&ManualPilot::publish_traj, this));

        // Start coordinates
        tx_ = -36.0; ty_ = 10.0; tz_ = 10.0;
        current_yaw_ = 3.14159; 
        
        lantern_reached = false;
        log_done = false;
        
        // Stability Anchor: Record start time to let physics settle
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Phase 1 Pilot Online. Waiting for physics to settle...");
    }

private:
    void auto_steer_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // Prevent movement for first 2 seconds of node life to stop rolling
        if ((this->now() - start_time_).seconds() < 2.0) return;
        
        if (msg->z < 0.5 || lantern_reached) return;

        double stop_threshold = 1600000.0; // Based on your logged data
        double turn_gain = 0.0015;
        double speed_gain = 0.08; 

        // Rotation Logic (Centering)
        current_yaw_ -= msg->x * turn_gain;

        // Forward Logic (Braking by Area)
        if (msg->y < stop_threshold) {
            tx_ += std::cos(current_yaw_) * speed_gain;
            ty_ += std::sin(current_yaw_) * speed_gain;
        } else {
            lantern_reached = true;
            reach_time = this->now();
            RCLCPP_INFO(this->get_logger(), "Lantern 1 Reached! Braking for 2 seconds...");
        }
    }

    void publish_traj() {
        if (lantern_reached && !log_done) {
            auto current_time = this->now();
            if ((current_time - reach_time).seconds() >= 2.0) {
                RCLCPP_INFO(this->get_logger(), "--- COORDINATE LOG ---");
                RCLCPP_INFO(this->get_logger(), "Lantern 1 found at X: %.2f, Y: %.2f", tx_, ty_);
                log_done = true;
            }
        }

        auto message = trajectory_msgs::msg::MultiDOFJointTrajectory();
        message.header.stamp = this->now();
        message.header.frame_id = "world";

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        geometry_msgs::msg::Transform transform;
        
        transform.translation.x = tx_;
        transform.translation.y = ty_;
        transform.translation.z = tz_;

        // Hard-zero X and Y rotation to keep drone flat (Stops Rolling)
        transform.rotation.x = 0.0;
        transform.rotation.y = 0.0;
        transform.rotation.z = std::sin(current_yaw_ / 2.0);
        transform.rotation.w = std::cos(current_yaw_ / 2.0);

        point.transforms.push_back(transform);
        
        // Zero out velocities to ensure smooth station-keeping
        point.velocities.push_back(geometry_msgs::msg::Twist());
        point.accelerations.push_back(geometry_msgs::msg::Twist());
        
        point.time_from_start.nanosec = 100000000;
        message.points.push_back(point);
        traj_pub_->publish(message);
    }

    // Member Variables
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double tx_, ty_, tz_, current_yaw_; 
    bool lantern_reached, log_done;
    rclcpp::Time reach_time, start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualPilot>());
    rclcpp::shutdown();
    return 0;
}
