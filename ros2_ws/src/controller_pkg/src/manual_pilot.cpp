#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"

using namespace std::chrono_literals;

enum State { NAV_TO_ENTRANCE, SEARCHING_CAVE, HUNTING, LOGGING, EXPLORING_DEEPER, MISSION_COMPLETE };

struct Waypoint { double x; double y; double z; };

class ManualPilot : public rclcpp::Node {
public:
    ManualPilot() : Node("manual_pilot") {
        target_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/lantern/target_data", 10, std::bind(&ManualPilot::auto_steer_callback, this, std::placeholders::_1));
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/command/trajectory", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&ManualPilot::publish_traj, this));

        tx_ = -36.0; ty_ = 10.0; tz_ = 10.0; current_yaw_ = 3.14159;
        current_state_ = NAV_TO_ENTRANCE;
        start_time_ = this->now();
        
        entrance_ = {-315.14, 8.38, 14.99}; 
        RCLCPP_INFO(this->get_logger(), ">>> ACTION: Starting. Navigating to Entrance.");
    }

private:
    void change_state(State new_state, const std::string& action_msg) {
        if (current_state_ != new_state) {
            current_state_ = new_state;
            RCLCPP_INFO(this->get_logger(), ">>> PHASE CHANGE: %s", action_msg.c_str());
        }
    }

    void auto_steer_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        if ((this->now() - start_time_).seconds() < 2.0) return;
        if (current_state_ == LOGGING || current_state_ == EXPLORING_DEEPER) return;

        if (current_state_ == SEARCHING_CAVE && msg->z == 1.0) {
            change_state(HUNTING, "Lantern Spotted. Moving to lock-on.");
        }

        if (current_state_ == HUNTING) {
            if (msg->z != 1.0) {
                change_state(SEARCHING_CAVE, "Visual lost. Resuming 3D safety search.");
                return;
            }
            double stop_threshold = 1200000.0;
            current_yaw_ -= msg->x * 0.0015;
            if (msg->y < stop_threshold) {
                tx_ += std::cos(current_yaw_) * 0.12;
                ty_ += std::sin(current_yaw_) * 0.12;
                tz_ -= 0.1; // Gentle dive
            } else {
                reach_time_ = this->now();
                found_lanterns_.push_back({tx_, ty_, tz_});
                change_state(LOGGING, "Lantern Reached. Stopping to log coordinates.");
            }
        }
        else if (current_state_ == SEARCHING_CAVE && msg->z == 2.0) {
            current_yaw_ -= msg->x * 0.012; // Wall avoidance
            tz_ += msg->y * 0.01;           // Floor/Ceiling avoidance
        }
    }

    void publish_traj() {
        if (current_state_ == NAV_TO_ENTRANCE) {
            double angle = std::atan2(entrance_.y - ty_, entrance_.x - tx_);
            current_yaw_ = angle;
            tx_ += std::cos(current_yaw_) * 0.25;
            ty_ += std::sin(current_yaw_) * 0.25;
            if (tz_ < entrance_.z) tz_ += 0.1;

            if (std::sqrt(std::pow(entrance_.x-tx_,2)+std::pow(entrance_.y-ty_,2)) < 3.0) {
                change_state(SEARCHING_CAVE, "In Cave. Autonomous exploration enabled.");
            }
        }
        else if (current_state_ == SEARCHING_CAVE) {
            tx_ += std::cos(current_yaw_) * 0.08;
            ty_ += std::sin(current_yaw_) * 0.08;
        }
        else if (current_state_ == LOGGING) {
            if ((this->now() - reach_time_).seconds() >= 2.0) {
                RCLCPP_INFO(this->get_logger(), ">>> DATA: L%ld logged at X:%.2f Y:%.2f Z:%.2f", 
                            found_lanterns_.size(), tx_, ty_, tz_);
                
                if (found_lanterns_.size() >= 4) {
                    change_state(MISSION_COMPLETE, "All 4 cave lanterns found. Parking.");
                } else {
                    change_state(EXPLORING_DEEPER, "Clearing area for 8s to prevent re-logging...");
                    reach_time_ = this->now();
                }
            }
        }
        else if (current_state_ == EXPLORING_DEEPER) {
            tx_ += std::cos(current_yaw_) * 0.15;
            ty_ += std::sin(current_yaw_) * 0.15;
            if (tz_ < 10.0) tz_ += 0.1; // Recovery climb

            if ((this->now() - reach_time_).seconds() >= 8.0) {
                change_state(SEARCHING_CAVE, "Area clear. Resuming search.");
            }
        }
        send_command();
    }

    void send_command() {
        auto message = trajectory_msgs::msg::MultiDOFJointTrajectory();
        message.header.stamp = this->now();
        message.header.frame_id = "world";
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        geometry_msgs::msg::Transform transform;
        transform.translation.x = tx_; transform.translation.y = ty_; transform.translation.z = tz_;
        transform.rotation.z = std::sin(current_yaw_ / 2.0); transform.rotation.w = std::cos(current_yaw_ / 2.0);
        point.transforms.push_back(transform);
        point.velocities.push_back(geometry_msgs::msg::Twist());
        point.accelerations.push_back(geometry_msgs::msg::Twist());
        point.time_from_start.nanosec = 100000000;
        message.points.push_back(point);
        traj_pub_->publish(message);
    }

    std::vector<Waypoint> found_lanterns_;
    Waypoint entrance_;
    State current_state_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double tx_, ty_, tz_, current_yaw_; 
    rclcpp::Time reach_time_, start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualPilot>());
    rclcpp::shutdown();
    return 0;
}
