#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"

using namespace std::chrono_literals;

class ManualPilot : public rclcpp::Node {
public:
    ManualPilot() : Node("manual_pilot") {
        // 1. Keyboard Subscriber
        kb_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ManualPilot::kb_callback, this, std::placeholders::_1));
        
        // 2. Publisher to the simulation controller
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "/command/trajectory", 10);

        // 3. 10Hz Timer for steady updates
        timer_ = this->create_wall_timer(100ms, std::bind(&ManualPilot::publish_traj, this));

        // Start coordinates
        tx_ = -36.0; ty_ = 10.0; tz_ = 10.0;
        current_yaw_ = 3.14159; 
        
        RCLCPP_INFO(this->get_logger(), "Survey Mode Online. Fly to the cave mouth and note the coordinates.");
    }

private:
    void kb_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double move_step = 0.6;
        double yaw_step = 0.2;

        current_yaw_ += msg->angular.z * yaw_step;

        double local_x = msg->linear.x * move_step;
        double local_y = msg->linear.y * move_step;

        tx_ += local_x * std::cos(current_yaw_) - local_y * std::sin(current_yaw_);
        ty_ += local_x * std::sin(current_yaw_) + local_y * std::cos(current_yaw_);
        tz_ += msg->linear.z * move_step;

        // --- SURVEY LOG ---
        // This will print every time you press a movement key
        RCLCPP_INFO(this->get_logger(), "CURRENT POS -> X: %.2f | Y: %.2f | Z: %.2f | Yaw: %.2f", 
                    tx_, ty_, tz_, current_yaw_);
    }

    void publish_traj() {
        auto message = trajectory_msgs::msg::MultiDOFJointTrajectory();
        message.header.stamp = this->now();
        message.header.frame_id = "world";

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        geometry_msgs::msg::Transform transform;
        
        transform.translation.x = tx_;
        transform.translation.y = ty_;
        transform.translation.z = tz_;

        // Stability: Forced level flight
        transform.rotation.x = 0.0;
        transform.rotation.y = 0.0;
        transform.rotation.z = std::sin(current_yaw_ / 2.0);
        transform.rotation.w = std::cos(current_yaw_ / 2.0);

        point.transforms.push_back(transform);
        point.velocities.push_back(geometry_msgs::msg::Twist());
        point.accelerations.push_back(geometry_msgs::msg::Twist());
        point.time_from_start.nanosec = 100000000;

        message.points.push_back(point);
        traj_pub_->publish(message);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr kb_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double tx_, ty_, tz_, current_yaw_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualPilot>());
    rclcpp::shutdown();
    return 0;
}
