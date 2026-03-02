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

enum class NodeType { PATH, FORK };

struct AreaNode {
    int id;
    NodeType type;
    octomap::point3d position;
    int parent_id = -1;
    bool left_taken = false;
    bool right_taken = false;
    bool is_closed = false; 
};

enum class ExplorerMode { EXPLORING, BACKTRACKING };

struct FrontierCandidates {
    octomap::point3d left_avg;
    octomap::point3d right_avg;
    bool has_left = false;
    bool has_right = false;
};

class SappuFrontierExplorer : public rclcpp::Node {
public:
    SappuFrontierExplorer() : Node("sappu_frontier_explorer"), 
                               mode_(ExplorerMode::EXPLORING), 
                               is_active_(false), 
                               has_odom_(false),
                               last_fork_id_(-1) {
        
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { is_active_ = msg->data; });

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_ = octomap::point3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
                has_odom_ = true;
                tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
                double r, p; tf2::Matrix3x3(q).getRPY(r, p, current_yaw_);
            });
        
        sub_map_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 1, [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
                octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                if (tree) octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
            });

        pub_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        timer_ = this->create_wall_timer(800ms, std::bind(&SappuFrontierExplorer::plannerLoop, this));
    }

private:
    void plannerLoop() {
        if (!is_active_ || !octree_ || !has_odom_) return;

        if (mode_ == ExplorerMode::EXPLORING && isPathBlocked(1.8)) {
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY: Wall within 1.8m. Backtracking.");
            mode_ = ExplorerMode::BACKTRACKING;
            if (!graph_.empty()) graph_.back().is_closed = true;
            return;
        }

        if (mode_ == ExplorerMode::EXPLORING) handleExploration();
        else handleBacktracking();
    }

    void handleExploration() {
        std::vector<octomap::point3d> frontiers = getGlobalFrontiers();
        FrontierCandidates candidates = splitFrontiers(frontiers);
        octomap::point3d target_pt;
        bool is_actual_fork = false;

        if (candidates.has_left && candidates.has_right) {
            // PROBE LOGIC: Look 4 meters ahead between candidates to see the fork wall early
            for (double step = 0.2; step <= 0.8; step += 0.1) {
                octomap::point3d check_pt = candidates.left_avg * step + candidates.right_avg * (1.0 - step);
                // Check if this point or anything 1.5m behind it is a wall
                for (double depth = 0.0; depth <= 4.0; depth += 0.5) {
                    octomap::point3d probe = current_pos_ + (check_pt - current_pos_).normalized() * depth;
                    auto node = octree_->search(probe);
                    if (node != nullptr && octree_->isNodeOccupied(node)) {
                        is_actual_fork = true; break;
                    }
                }
                if (is_actual_fork) break;
            }

            if (is_actual_fork) {
                // Prevent duplicate fork nodes if we already made one nearby
                bool too_close_to_last_fork = false;
                if (last_fork_id_ != -1 && (current_pos_ - graph_[last_fork_id_].position).norm() < 10.0) {
                    too_close_to_last_fork = true;
                }

                if (!too_close_to_last_fork) {
                    createNewNode(true);
                    last_fork_id_ = graph_.size() - 1;
                }

                // Sharp nudge to clear the corner
                double nudge_angle = current_yaw_ + (M_PI / 3.0); 
                target_pt = octomap::point3d(
                    current_pos_.x() + 3.0 * cos(nudge_angle),
                    current_pos_.y() + 3.0 * sin(nudge_angle),
                    current_pos_.z()
                );
                RCLCPP_INFO(this->get_logger(), "Fork Predicted Ahead. Executing sharp Left Nudge.");
            } else {
                target_pt = (candidates.left_avg + candidates.right_avg) * 0.5;
            }
        } else if (candidates.has_left) {
            target_pt = candidates.left_avg;
        } else if (candidates.has_right) {
            target_pt = candidates.right_avg;
        } else {
            mode_ = ExplorerMode::BACKTRACKING;
            return;
        }

        if (graph_.empty() || ((current_pos_ - graph_.back().position).norm() > 7.0 && !is_actual_fork)) {
            createNewNode(false);
        }

        checkLoopClosure();
        publishTarget(target_pt);
    }

    void handleBacktracking() {
        if (graph_.empty()) return;
        AreaNode& curr = graph_.back();
        auto candidates = splitFrontiers(getGlobalFrontiers());

        if (curr.type == NodeType::FORK && candidates.has_right && !curr.right_taken) {
            curr.right_taken = true;
            mode_ = ExplorerMode::EXPLORING;
            publishTarget(candidates.right_avg);
        } else {
            if (curr.parent_id == -1) { is_active_ = false; return; }
            octomap::point3d p_pos = graph_[curr.parent_id].position;
            publishTarget(p_pos);
            if ((current_pos_ - p_pos).norm() < 2.0) graph_.pop_back();
        }
    }

    bool isPathBlocked(double dist) {
        for (double a = -0.3; a <= 0.3; a += 0.3) {
            octomap::point3d p(current_pos_.x() + dist * cos(current_yaw_ + a),
                               current_pos_.y() + dist * sin(current_yaw_ + a),
                               current_pos_.z());
            auto n = octree_->search(p);
            if (n != nullptr && octree_->isNodeOccupied(n)) return true;
        }
        return false;
    }

    FrontierCandidates splitFrontiers(const std::vector<octomap::point3d>& frontiers) {
        FrontierCandidates c;
        std::vector<octomap::point3d> lg, rg;
        for (const auto& f : frontiers) {
            double angle = atan2(f.y() - current_pos_.y(), f.x() - current_pos_.x());
            double rel = angle - current_yaw_;
            while (rel > M_PI) rel -= 2.0 * M_PI;
            while (rel < -M_PI) rel += 2.0 * M_PI;
            if (std::abs(rel) < (M_PI / 2.0)) {
                if (rel > 0.15) lg.push_back(f);
                else if (rel < -0.15) rg.push_back(f);
            }
        }
        auto avg = [](const std::vector<octomap::point3d>& v) {
            octomap::point3d s(0,0,0); for (auto p : v) s += p; return s * (1.0/v.size());
        };
        if (!lg.empty()) { c.left_avg = avg(lg); c.has_left = true; }
        if (!rg.empty()) { c.right_avg = avg(rg); c.has_right = true; }
        return c;
    }

    void createNewNode(bool fork) {
        AreaNode n;
        n.id = graph_.size();
        n.position = current_pos_;
        n.parent_id = graph_.empty() ? -1 : graph_.back().id;
        n.type = fork ? NodeType::FORK : NodeType::PATH;
        n.left_taken = fork;
        graph_.push_back(n);
        RCLCPP_INFO(this->get_logger(), "Node %d Created (%s)", n.id, (fork ? "FORK" : "PATH"));
    }

    void checkLoopClosure() {
        if (graph_.size() < 4) return;
        for (size_t i = 0; i < graph_.size() - 3; ++i) {
            if ((current_pos_ - graph_[i].position).norm() < 6.0) {
                mode_ = ExplorerMode::BACKTRACKING; break;
            }
        }
    }

    std::vector<octomap::point3d> getGlobalFrontiers() {
        std::vector<octomap::point3d> frs; if (!octree_) return frs;
        double r = octree_->getResolution();
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (!octree_->isNodeOccupied(*it)) {
                octomap::point3d p = it.getCoordinate();
                octomap::point3d n[6] = {{p.x()+r,p.y(),p.z()}, {p.x()-r,p.y(),p.z()}, {p.x(),p.y()+r,p.z()}, 
                                         {p.x(),p.y()-r,p.z()}, {p.x(),p.y(),p.z()+r}, {p.x(),p.y(),p.z()-r}};
                for(int i=0; i<6; i++) if(octree_->search(n[i]) == nullptr) { frs.push_back(p); break; }
            }
        }
        return frs;
    }

    void publishTarget(octomap::point3d t) {
        auto m = geometry_msgs::msg::PoseStamped();
        m.header.stamp = this->now(); m.header.frame_id = "world";
        m.pose.position.x = t.x(); m.pose.position.y = t.y(); m.pose.position.z = t.z();
        double y = atan2(t.y() - current_pos_.y(), t.x() - current_pos_.x());
        tf2::Quaternion q; q.setRPY(0, 0, y);
        m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z(); m.pose.orientation.w = q.w();
        pub_target_->publish(m);
    }

    ExplorerMode mode_;
    std::vector<AreaNode> graph_;
    std::shared_ptr<octomap::OcTree> octree_;
    octomap::point3d current_pos_;
    double current_yaw_;
    bool is_active_, has_odom_;
    int last_fork_id_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SappuFrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}