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

// --- TUNABLE PARAMETERS ---
const int MIN_CLUSTER_SIZE = 150;           // Minimum frontiers to be a valid goal
const double CLUSTER_DIST_THRESHOLD = 1.2;  // Distance for grouping frontier voxels
const double TARGET_DISTANCE_OFFSET = 4.0;  // Stop 3m before the cluster center
const double REACHED_THRESHOLD = 1.2;       // Distance to consider setpoint reached
const double FORBIDDEN_RADIUS = 8.0;        // Ignore frontiers near previous graph nodes
const double HEADING_BIAS = 2.0;            // Score multiplier for clusters in front
// --------------------------

struct GraphNode {
    int id;
    octomap::point3d position;
    int parent_id = -1;
};

enum class State { EXPLORING, BACKTRACKING };

class SappuFrontierExplorer : public rclcpp::Node {
public:
    SappuFrontierExplorer() : Node("sappu_frontier_explorer"), state_(State::EXPLORING), is_active_(false), has_target_(false) {
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&SappuFrontierExplorer::odomCallback, this, std::placeholders::_1));

        sub_map_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 1, [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
                octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                if (tree) octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
            });

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { 
                is_active_ = msg->data;
                // RCLCPP_INFO(this->get_logger(), "Exploration %s", is_active_ ? "ENABLED" : "DISABLED");
            });

        pub_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        
        RCLCPP_INFO(this->get_logger(), "Sappu Frontier Explorer v2 (Event-Driven) Ready.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pos_ = octomap::point3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double r, p; tf2::Matrix3x3(q).getRPY(r, p, current_yaw_);
        has_odom_ = true;

        if (!is_active_ || !octree_) return;

        // Trigger planning if no target exists or if the current one is reached
        if (!has_target_ || (current_pos_ - current_target_).norm() < REACHED_THRESHOLD) {
            RCLCPP_INFO(this->get_logger(), "Target reached. Recomputing goal...");
            computeNextGoal();
        }
    }

    void computeNextGoal() {
        if (graph_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Initializing Graph at Start Position.");
            addNode(); 
        }

        auto clusters = findFrontierClusters();

        if (state_ == State::EXPLORING) {
            bool found_reachable_cluster = false;
            octomap::point3d chosen_cluster;

            for (const auto& cluster_center : clusters) {
                // Check if we can actually fly to this cluster's candidate point
                octomap::point3d direction = (cluster_center - current_pos_).normalized();
                octomap::point3d candidate_target = cluster_center - (direction * TARGET_DISTANCE_OFFSET);

                if (isPathClear(current_pos_, candidate_target)) {
                    chosen_cluster = cluster_center;
                    current_target_ = candidate_target;
                    found_reachable_cluster = true;
                    break; // Found the best possible reachable goal!
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "Best cluster blocked by wall, checking next...");
                }
            }

            if (!found_reachable_cluster) {
                RCLCPP_WARN(this->get_logger(), "No clusters are reachable via straight line. Backtracking...");
                state_ = State::BACKTRACKING;
                return; 
            }

            RCLCPP_INFO(this->get_logger(), "EXPLORING: Heading to reachable cluster at [%.1f, %.1f]", chosen_cluster.x(), chosen_cluster.y());
            addNode(); 
            publishTarget(current_target_);
            has_target_ = true;
        }
        else { // BACKTRACKING
            if (!clusters.empty()) {
                RCLCPP_INFO(this->get_logger(), "Found new path during backtracking! Switching to EXPLORING.");
                state_ = State::EXPLORING;
                computeNextGoal(); 
                return;
            }

            if (graph_.size() <= 1) {
                is_active_ = false;
                RCLCPP_INFO(this->get_logger(), "BACK TO START. Mission Complete.");
                return;
            }

            // Move toward the parent of the current graph node
            int parent_idx = graph_.back().parent_id;
            current_target_ = graph_[parent_idx].position;
            
            RCLCPP_INFO(this->get_logger(), "BACKTRACKING: Moving to Node %d at [%.1f, %.1f]", parent_idx, current_target_.x(), current_target_.y());
            
            publishTarget(current_target_);
            has_target_ = true;
            
            // Pop the node once we reach its parent
            if ((current_pos_ - current_target_).norm() < REACHED_THRESHOLD) {
                graph_.pop_back();
            }
        }
    }

    bool isPathClear(octomap::point3d start, octomap::point3d end) {
        octomap::point3d direction = (end - start).normalized();
        double dist = (end - start).norm();
        octomap::point3d hit_point;
        // Raycast from current position to target. If it hits a wall before reaching target, path is blocked.
        if (octree_->castRay(start, direction, hit_point, true, dist)) {
            return false; // Hit an occupied voxel
        }
        return true; 
    }

    std::vector<octomap::point3d> findFrontierClusters() {
        std::vector<octomap::point3d> frontiers;
        double res = octree_->getResolution();

        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (!octree_->isNodeOccupied(*it)) {
                octomap::point3d p = it.getCoordinate();
                
                // IGNORE ENTRANCE: Check distance to all previous graph nodes
                bool too_close_to_history = false;
                for (const auto& node : graph_) {
                    if ((p - node.position).norm() < FORBIDDEN_RADIUS) {
                        too_close_to_history = true;
                        break;
                    }
                }
                if (too_close_to_history) continue;

                // Detect if node is a frontier (has unknown neighbor)
                bool is_frontier = false;
                octomap::point3d neighbors[6] = {
                    {p.x()+res, p.y(), p.z()}, {p.x()-res, p.y(), p.z()},
                    {p.x(), p.y()+res, p.z()}, {p.x(), p.y()-res, p.z()},
                    {p.x(), p.y(), p.z()+res}, {p.x(), p.y(), p.z()-res}
                };
                for(auto& n : neighbors) {
                    if (octree_->search(n) == nullptr) { is_frontier = true; break; }
                }
                if (is_frontier) frontiers.push_back(p);
            }
        }

        if (frontiers.empty()) return {};

        // Sort for clustering stability
        std::sort(frontiers.begin(), frontiers.end(), [](auto& a, auto& b){ return a.x() < b.x(); });

        std::vector<std::vector<octomap::point3d>> clusters;
        for (auto& f : frontiers) {
            bool found = false;
            for (auto& cluster : clusters) {
                if ((f - cluster.back()).norm() < CLUSTER_DIST_THRESHOLD) {
                    cluster.push_back(f);
                    found = true; break;
                }
            }
            if (!found) clusters.push_back({f});
        }

        // Scoring: Size + Heading Bias
        std::vector<std::pair<double, octomap::point3d>> scored_clusters;
        for (auto& c : clusters) {
            if (c.size() >= (size_t)MIN_CLUSTER_SIZE) {
                octomap::point3d avg(0,0,0);
                for (auto& p : c) avg += p;
                octomap::point3d center = avg * (1.0/c.size());

                if (!isPathClear(current_pos_, center)) continue; // Skip this cluster if a wall is in the way

                double angle = atan2(center.y() - current_pos_.y(), center.x() - current_pos_.x());
                double rel_angle = std::remainder(angle - current_yaw_, 2 * M_PI);
                
                double score = (double)c.size();
                if (std::abs(rel_angle) < (M_PI / 2.0)) score *= HEADING_BIAS; 

                scored_clusters.push_back({score, center});
            }
        }

        std::sort(scored_clusters.rbegin(), scored_clusters.rend());
        
        RCLCPP_INFO(this->get_logger(), "Found %zu clusters. Best score: %.1f (Size: ~%zu)", 
                    scored_clusters.size(), scored_clusters.empty() ? 0.0 : scored_clusters[0].first, 
                    scored_clusters.empty() ? 0 : (size_t)(scored_clusters[0].first / (scored_clusters[0].first > (double)MIN_CLUSTER_SIZE * HEADING_BIAS ? HEADING_BIAS : 1.0)));

        std::vector<octomap::point3d> result;
        for (auto& sc : scored_clusters) result.push_back(sc.second);
        return result;
    }

    void addNode() {
        GraphNode n;
        n.id = graph_.size();
        n.position = current_pos_;
        n.parent_id = graph_.empty() ? -1 : graph_.back().id;
        graph_.push_back(n);
        RCLCPP_INFO(this->get_logger(), ">>> NODE %d CREATED at [%.1f, %.1f]", n.id, n.position.x(), n.position.y());
    }

    void publishTarget(octomap::point3d t) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";
        msg.pose.position.x = t.x();
        msg.pose.position.y = t.y();
        msg.pose.position.z = t.z();
        
        double yaw = atan2(t.y() - current_pos_.y(), t.x() - current_pos_.x());
        tf2::Quaternion q; q.setRPY(0, 0, yaw);
        msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
        
        pub_target_->publish(msg);
    }

    State state_;
    std::vector<GraphNode> graph_;
    std::shared_ptr<octomap::OcTree> octree_;
    octomap::point3d current_pos_, current_target_;
    double current_yaw_;
    bool is_active_, has_odom_ = false, has_target_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SappuFrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}