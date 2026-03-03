#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class SmartExplorer : public rclcpp::Node {
public:
    SmartExplorer() : Node("smart_explorer") {
        auto octomap_qos = rclcpp::QoS(1).transient_local().reliable();
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_full", octomap_qos, std::bind(&SmartExplorer::octomap_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&SmartExplorer::pose_callback, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&SmartExplorer::explore_step, this));
        
        RCLCPP_INFO(this->get_logger(), "Advanced Frontier Explorer started.");
    }

private:
    enum class State { EXPLORING, BACKTRACKING };
    State current_state_ = State::EXPLORING;
    
    std::shared_ptr<octomap::OcTree> octree_;
    nav_msgs::msg::Odometry current_odom_;
    bool odom_received_ = false;

    // Breadcrumbs to allow backtracking
    std::vector<geometry_msgs::msg::Point> breadcrumbs_;
    double breadcrumb_spacing_ = 4.0; 

    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        odom_received_ = true;
        
        // Drop breadcrumbs as we move
        if (current_state_ == State::EXPLORING) {
            if (breadcrumbs_.empty() || get_dist(current_odom_.pose.pose.position, breadcrumbs_.back()) > breadcrumb_spacing_) {
                breadcrumbs_.push_back(current_odom_.pose.pose.position);
            }
        }
    }

    void explore_step() {
    if (!octree_ || !odom_received_) return;

    auto pos = current_odom_.pose.pose.position;
    
    if (current_state_ == State::EXPLORING) {
        RCLCPP_INFO(this->get_logger(), "--- START FAN SCAN ---");
        RCLCPP_INFO(this->get_logger(), "Current Pos: (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);

        double best_yaw = 0;
        double max_depth = 0;
        bool found_opening = false;

        tf2::Quaternion q;
        tf2::fromMsg(current_odom_.pose.pose.orientation, q);
        double roll, pitch, current_yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);
        RCLCPP_INFO(this->get_logger(), "Current Yaw: %.2f rad", current_yaw);

        // Scan the fan
        for (double angle = -1.0; angle <= 1.0; angle += 0.2) {
            double probe_yaw = current_yaw + angle;
            double depth = get_ray_depth(pos, probe_yaw);
            
            // PRINT: See the depth of every single ray in the fan
            RCLCPP_INFO(this->get_logger(), "  > Probe Yaw: %.2f | Depth: %.2f m", probe_yaw, depth);
            
            if (depth > max_depth) {
                max_depth = depth;
                best_yaw = probe_yaw;
                found_opening = true;
            }
        }

        // 2. Dead-end check
        if (!found_opening || max_depth < 3.0) {
            RCLCPP_WARN(this->get_logger(), "!!! DEAD END !!! Max depth only %.2f m. Backtracking...", max_depth);
            current_state_ = State::BACKTRACKING;
            return;
        }

        // 3. Selection Debug
        RCLCPP_INFO(this->get_logger(), "CHOSEN: Yaw %.2f with Depth %.2f", best_yaw, max_depth);

        // Use a safety margin (subtracting 1.5m) to avoid hitting the wall
        double safety_margin = 1.5; 
        double move_dist = std::max(1.0, max_depth - safety_margin);
        
        double gx = pos.x + move_dist * cos(best_yaw);
        double gy = pos.y + move_dist * sin(best_yaw);
        
        RCLCPP_INFO(this->get_logger(), "NOT PUBLISHING GOAL: (%.2f, %.2f)", gx, gy);
        // publish_goal(gx, gy, pos.z, best_yaw);
    }
        else { // BACKTRACKING MODE
            if (breadcrumbs_.empty()) {
                current_state_ = State::EXPLORING;
                return;
            }

            auto target = breadcrumbs_.back();
            if (get_dist(pos, target) < 1.5) {
                breadcrumbs_.pop_back(); // We reached this crumb, try the previous one
            } else {
                publish_goal(target.x, target.y, target.z, atan2(target.y - pos.y, target.x - pos.x));
            }
        }
    }

    double get_ray_depth(geometry_msgs::msg::Point start, double yaw) {
        octomap::point3d origin(start.x, start.y, start.z);
        octomap::point3d direction(cos(yaw), sin(yaw), 0);
        octomap::point3d end;
        
        // Raycast until we hit a wall or reach max sensor range
        if (octree_->castRay(origin, direction, end, true, 15.0)) {
            return (end - origin).norm();
        }
        return 15.0; // Assume clear path if nothing hit
    }

    double get_dist(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b) {
        return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2) + pow(a.z-b.z,2));
    }

    void publish_goal(double x, double y, double z, double yaw) {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "world";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.orientation = tf2::toMsg(q);
        goal_pub_->publish(goal);
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartExplorer>());
    rclcpp::shutdown();
    return 0;
}