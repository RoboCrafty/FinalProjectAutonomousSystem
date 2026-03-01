#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

struct JunctionNode {
    octomap::point3d position;
    bool explored;
};

class FrontierExplorerNode : public rclcpp::Node {
public:
    FrontierExplorerNode() : Node("frontier_explorer_node"), is_exploring_(false), has_odom_(false) {
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>("/enable_exploration", 10, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) { is_exploring_ = msg->data; });

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/current_state_est", 10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_ = octomap::point3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
                has_odom_ = true;
                tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
                double r, p; tf2::Matrix3x3(q).getRPY(r, p, current_yaw_);
            });
        
        sub_map_ = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", 1, 
            [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
                octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                if (tree) octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
            });

        pub_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        timer_ = this->create_wall_timer(800ms, std::bind(&FrontierExplorerNode::plannerLoop, this));
    }

private:
    void plannerLoop() {
        if (!is_exploring_ || !octree_ || !has_odom_) return;

        std::vector<octomap::point3d> local_frontiers;
        std::vector<octomap::point3d> global_frontiers;
        double res = octree_->getResolution();

        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (!octree_->isNodeOccupied(*it)) {
                octomap::point3d p = it.getCoordinate();
                if (isFrontier(p, res)) {
                    global_frontiers.push_back(p);
                    if (isIn180FOV(p)) local_frontiers.push_back(p);
                }
            }
        }

        octomap::point3d target_pt;
        bool found = false;

        if (!local_frontiers.size() == 0) {
            // 1. Calculate Standard Centroid
            octomap::point3d sum(0,0,0);
            for (const auto& f : local_frontiers) sum += f;
            target_pt = sum * (1.0 / local_frontiers.size());
            found = true;

            // 2. Junction Detection & Nudging
            if (isAtFork(local_frontiers)) {
                saveJunction();
                // NUDGE LOGIC: Offset the target to the left (approx 30 degrees) to avoid the wall
                double dist_to_target = (target_pt - current_pos_).norm();
                double angle_to_target = atan2(target_pt.y() - current_pos_.y(), target_pt.x() - current_pos_.x());
                double nudged_angle = angle_to_target + (M_PI / 6.0); // +30 degrees nudge
                
                target_pt.x() = current_pos_.x() + dist_to_target * cos(nudged_angle);
                target_pt.y() = current_pos_.y() + dist_to_target * sin(nudged_angle);
                RCLCPP_INFO(this->get_logger(), "Fork detected: Nudging target left to avoid center wall.");
            }
        } 
        else {
            // 3. LIFO GRAPH BACKTRACKING
            for (int i = junction_stack_.size() - 1; i >= 0; --i) {
                octomap::point3d j_pos = junction_stack_[i].position;
                for (const auto& gf : global_frontiers) {
                    if ((gf - j_pos).norm() < 15.0) { // Search radius around old junction
                        target_pt = gf;
                        found = true;
                        break;
                    }
                }
                if (found) {
                    RCLCPP_WARN(this->get_logger(), "Ring/Dead-end reached. Backtracking to Junction %d.", i);
                    break;
                }
            }
        }

        if (found) publishTarget(target_pt);
    }

    bool isAtFork(const std::vector<octomap::point3d>& local) {
        if (local.size() < 20) return false;
        // Check for high variance/spread in frontiers which indicates a split
        double max_d = 0;
        for (size_t i = 0; i < local.size(); i += 5) {
            double d = (local[0] - local[i]).norm();
            if (d > max_d) max_d = d;
        }
        return max_d > 10.0; // Distance threshold for fork detection
    }

    void saveJunction() {
        bool exists = false;
        for (const auto& j : junction_stack_) {
            if ((j.position - current_pos_).norm() < 10.0) { exists = true; break; }
        }
        if (!exists) {
            junction_stack_.push_back({current_pos_, false});
            RCLCPP_INFO(this->get_logger(), "New Junction Node saved to Graph.");
        }
    }

    bool isIn180FOV(octomap::point3d p) {
        double angle = atan2(p.y() - current_pos_.y(), p.x() - current_pos_.x());
        double diff = angle - current_yaw_;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return std::abs(diff) < (M_PI / 2.0);
    }

    bool isFrontier(octomap::point3d p, double res) {
        octomap::point3d n[6] = {{p.x()+res,p.y(),p.z()}, {p.x()-res,p.y(),p.z()}, {p.x(),p.y()+res,p.z()}, 
                                 {p.x(),p.y()-res,p.z()}, {p.x(),p.y()+res,p.z()}, {p.x(),p.y(),p.z()-res}};
        for(int i=0; i<6; i++) if(octree_->search(n[i]) == nullptr) return true;
        return false;
    }

    void publishTarget(octomap::point3d t) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now(); msg.header.frame_id = "world";
        msg.pose.position.x = t.x(); msg.pose.position.y = t.y(); msg.pose.position.z = t.z();
        double yaw = atan2(t.y() - current_pos_.y(), t.x() - current_pos_.x());
        tf2::Quaternion q; q.setRPY(0, 0, yaw);
        msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
        pub_target_->publish(msg);
    }

    std::vector<JunctionNode> junction_stack_;
    std::shared_ptr<octomap::OcTree> octree_;
    octomap::point3d current_pos_;
    double current_yaw_;
    bool is_exploring_, has_odom_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorerNode>());
    rclcpp::shutdown();
    return 0;
}