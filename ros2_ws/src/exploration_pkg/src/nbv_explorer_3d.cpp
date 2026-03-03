#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>
#include <cmath>
#include <stack> // NEW: Added for Breadcrumb Memory

using namespace std::chrono_literals;

// NEW: Data structure for our memory
struct Breadcrumb {
    double x, y, z;
};

class NBVExplorer3D : public rclcpp::Node {
public:
    NBVExplorer3D() : Node("nbv_explorer_3d_node"), is_exploring_(false), target_x_(0), target_y_(0), target_z_(0), has_target_(false) {
        
        // Subscribers
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10, std::bind(&NBVExplorer3D::enableCb, this, std::placeholders::_1));
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&NBVExplorer3D::odomCb, this, std::placeholders::_1));
        
        // Subscribe to the TRUE 3D Binary Map
        sub_map_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&NBVExplorer3D::mapCb, this, std::placeholders::_1));

        // Publisher
        pub_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);

        // Timer
        timer_ = this->create_wall_timer(500ms, std::bind(&NBVExplorer3D::explorationLoop, this));

        // Initialize the last dropped crumb
        last_dropped_ = {0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "3D NBV Explorer (With Breadcrumbs) Initialized. Waiting for Activation...");
    }

private:
    void enableCb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !is_exploring_) {
            RCLCPP_INFO(this->get_logger(), "3D EXPLORATION ACTIVATED! Taking control of the cavern.");
            is_exploring_ = true;
        }
    }

    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void mapCb(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
        }
    }

    double scoreCandidate(double cx, double cy, double cz, double cand_yaw, double distance) {
        if (!octree_) return -9999.0;

        // Line-of-Sight Raycasting
        octomap::point3d origin(current_x_, current_y_, current_z_);
        octomap::point3d direction(cx - current_x_, cy - current_y_, cz - current_z_);
        octomap::point3d hit_pt;
        
        bool hit = octree_->castRay(origin, direction, hit_pt, true, distance);
        if (hit) {
            return -9999.0; 
        }

        // 1. Safety Check: 3D Spherical Forcefield (User Tuned: 5.0m radius)
        double safe_r = 5.0;
        double res = octree_->getResolution();
        if (res <= 0.0) res = 0.2; 
        
        double step_safe = res * 2.0; 
        for (double dx = -safe_r; dx <= safe_r; dx += step_safe) {
            for (double dy = -safe_r; dy <= safe_r; dy += step_safe) {
                for (double dz = -safe_r; dz <= safe_r; dz += step_safe) {
                    if (dx*dx + dy*dy + dz*dz > safe_r*safe_r) continue; 
                    
                    octomap::OcTreeNode* node = octree_->search(cx + dx, cy + dy, cz + dz);
                    if (node && octree_->isNodeOccupied(node)) {
                        return -9999.0; 
                    }
                }
            }
        }

        // 2. Information Gain
        double score = 0.0;
        double search_r = 18.0; 
        double step_search = res * 4.0; 
        for (double dx = -search_r; dx <= search_r; dx += step_search) {
            for (double dy = -search_r; dy <= search_r; dy += step_search) {
                for (double dz = -search_r; dz <= search_r; dz += step_search) {
                    if (dx*dx + dy*dy + dz*dz > search_r*search_r) continue;
                    
                    octomap::OcTreeNode* node = octree_->search(cx + dx, cy + dy, cz + dz);
                    if (node == nullptr) {
                        score += 1.0; 
                    }
                }
            }
        }

        // 3. Straight-line bonus
        double yaw_diff = std::abs(cand_yaw - current_yaw_);
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
        score -= (std::abs(yaw_diff) * 20.0);

        return score;
    }

    void explorationLoop() {
        if (!is_exploring_ || !octree_) return;

        // Lookahead Planning
        if (has_target_) {
            double dist = std::sqrt(std::pow(target_x_ - current_x_, 2) + std::pow(target_y_ - current_y_, 2) + std::pow(target_z_ - current_z_, 2));
            if (dist > 6.0) return;
        }

        double best_score = -9999.0;
        double best_x = 0, best_y = 0, best_z = 0;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> yaw_dist(-1.2, 1.2);   
        std::uniform_real_distribution<> pitch_dist(-0.8, 0.8); 
        std::uniform_real_distribution<> dist_dist(6.0, 15.0);  

        // Sample 60 points in a 3D forward cone
        for (int i = 0; i < 60; ++i) {
            double angle_yaw = current_yaw_ + yaw_dist(gen);
            double angle_pitch = pitch_dist(gen);
            double distance = dist_dist(gen);

            double cand_x = current_x_ + distance * std::cos(angle_yaw) * std::cos(angle_pitch);
            double cand_y = current_y_ + distance * std::sin(angle_yaw) * std::cos(angle_pitch);
            double cand_z = current_z_ + distance * std::sin(angle_pitch);

            if (cand_z < -100.0) cand_z = -100.0;
            if (cand_z > 100.0) cand_z = 100.0;

            double score = scoreCandidate(cand_x, cand_y, cand_z, angle_yaw, distance);

            if (score > best_score) {
                best_score = score;
                best_x = cand_x;
                best_y = cand_y;
                best_z = cand_z;
            }
        }

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        if (best_score > 0) {
            // --- NEW: WE FOUND NEW SPACE! ---
            // Drop a breadcrumb if we moved more than 5 meters from the last one
            double dist_from_last = std::sqrt(std::pow(current_x_ - last_dropped_.x, 2) + 
                                              std::pow(current_y_ - last_dropped_.y, 2) + 
                                              std::pow(current_z_ - last_dropped_.z, 2));
            if (dist_from_last > 8.0 || breadcrumbs_.empty()) {
                breadcrumbs_.push({current_x_, current_y_, current_z_});
                last_dropped_ = {current_x_, current_y_, current_z_};
                RCLCPP_INFO(this->get_logger(), "Dropped Breadcrumb! Memory size: %zu", breadcrumbs_.size());
            }

            target_x_ = best_x; target_y_ = best_y; target_z_ = best_z;
            has_target_ = true;

            double target_yaw = std::atan2(best_y - current_y_, best_x - current_x_);
            tf2::Quaternion q;
            q.setRPY(0, 0, target_yaw);

            msg.pose.position.x = best_x;
            msg.pose.position.y = best_y;
            msg.pose.position.z = best_z;
            msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();

            RCLCPP_INFO(this->get_logger(), "3D Leap to: (%.1f, %.1f, %.1f) | Score: %.1f", best_x, best_y, best_z, best_score);
            pub_target_->publish(msg);
            
        } else {
            // --- NEW: DEAD END ESCAPE PLAN ---
            if (!breadcrumbs_.empty()) {
                Breadcrumb bc = breadcrumbs_.top();
                breadcrumbs_.pop(); // Erase the crumb so we don't get stuck in a loop

                RCLCPP_WARN(this->get_logger(), "Dead end! Reversing to Breadcrumb (%.1f, %.1f, %.1f). Left: %zu", bc.x, bc.y, bc.z, breadcrumbs_.size());

                target_x_ = bc.x;
                target_y_ = bc.y;
                target_z_ = bc.z;
                has_target_ = true;

                // Point the camera toward the breadcrumb as we fly there
                double target_yaw = std::atan2(target_y_ - current_y_, target_x_ - current_x_);
                tf2::Quaternion q;
                q.setRPY(0, 0, target_yaw);

                msg.pose.position.x = target_x_;
                msg.pose.position.y = target_y_;
                msg.pose.position.z = target_z_;
                msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
                msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();

                pub_target_->publish(msg);
            } else {
                // If the memory is completely empty, we either finished the whole map or are critically stuck.
                RCLCPP_WARN(this->get_logger(), "Map Fully Explored or No Breadcrumbs! Spinning...");
                
                double spin_yaw = current_yaw_ + 0.78;
                target_x_ = current_x_ + 1.0 * std::cos(spin_yaw);
                target_y_ = current_y_ + 1.0 * std::sin(spin_yaw);
                target_z_ = current_z_; 
                has_target_ = true;

                tf2::Quaternion q;
                q.setRPY(0, 0, spin_yaw);

                msg.pose.position.x = target_x_;
                msg.pose.position.y = target_y_;
                msg.pose.position.z = target_z_;
                msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
                msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();

                pub_target_->publish(msg);
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<octomap::OcTree> octree_;
    
    // NEW variables
    std::stack<Breadcrumb> breadcrumbs_;
    Breadcrumb last_dropped_;
    
    bool is_exploring_, has_target_;
    double current_x_, current_y_, current_z_, current_yaw_;
    double target_x_, target_y_, target_z_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NBVExplorer3D>());
    rclcpp::shutdown();
    return 0;
}