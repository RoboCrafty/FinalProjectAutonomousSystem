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
const int MIN_CLUSTER_SIZE = 150;           // Minimum number of frontier voxels to consider a valid cluster
const int ABSOLUTE_MIN_SIZE = 50;           // Absolute minimum cluster size to even consider (for dynamic scaling)
const double CLUSTER_DIST_THRESHOLD = 1.2;  // Distance for grouping frontier voxels
const double TARGET_DISTANCE_OFFSET = 4.0;  // Stop 4m before the cluster center
const double REACHED_THRESHOLD = 1.2;       // Distance to consider setpoint reached
const double FORBIDDEN_RADIUS = 8.0;        // Ignore frontiers near previous graph nodes
const double HEADING_BIAS = 2.0;            // Score multiplier for clusters in front
const double MAX_PLANNING_DIST = 40.0;      // Max distance to attempt a direct flight
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
        
        // Initialize dynamic threshold
        current_min_size_ = MIN_CLUSTER_SIZE;

        RCLCPP_INFO(this->get_logger(), "Sappu Frontier Explorer v3 (Dynamic Scaling) Ready.");
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

        if (state_ == State::EXPLORING) {
            bool found_goal = false;
            octomap::point3d chosen_cluster; // for debug porpuse only
            
            // Attempt to find clusters by gradually reducing the threshold
            while (current_min_size_ >= ABSOLUTE_MIN_SIZE) {
                // Get clusters based on the CURRENT dynamic threshold
                auto scored_clusters = findFrontierClusters(current_min_size_); 
                
                for (const auto& pair : scored_clusters) {
                    octomap::point3d center = pair.second;
                    
                    // Safety: Skip clusters that are too far away
                    if ((center - current_pos_).norm() > MAX_PLANNING_DIST) continue;

                    octomap::point3d dir = (center - current_pos_).normalized();
                    octomap::point3d candidate = center - (dir * TARGET_DISTANCE_OFFSET);

                    // Safety: Raycast check to ensure we don't fly through a wall
                    if (isPathClear(current_pos_, candidate)) {
                        current_target_ = candidate;
                        chosen_cluster = center; // for debug porpuse only
                        found_goal = true;
                        
                        // If we found a cluster, gradually recover the threshold for the next jump
                        current_min_size_ = std::min(MIN_CLUSTER_SIZE, current_min_size_ + 20);
                        RCLCPP_INFO(this->get_logger(), "Goal found with threshold: %d", current_min_size_);
                        break; 
                    }
                }
                
                if (found_goal) break;

                // No reachable clusters at this size, lower the bar and try again
                current_min_size_ -= 30; 
                RCLCPP_INFO(this->get_logger(), "No clusters found. Reducing threshold to: %d", current_min_size_);
            }

            if (!found_goal) {
                RCLCPP_WARN(this->get_logger(), "Even at minimum threshold, no paths found. Backtracking...");
                state_ = State::BACKTRACKING;
                current_min_size_ = MIN_CLUSTER_SIZE; // Reset for when we finish backtracking
                computeNextGoal(); // Immediately start moving back
                return;
            }

            RCLCPP_INFO(this->get_logger(), "EXPLORING: Heading to reachable cluster at [%.1f, %.1f]", chosen_cluster.x(), chosen_cluster.y());
            addNode();
            publishTarget(current_target_);
            has_target_ = true;
        }
        else { // BACKTRACKING
            // During backtracking, we ONLY switch back to exploring if we find a HIGH QUALITY cluster
            auto clusters = findFrontierClusters(MIN_CLUSTER_SIZE);

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

    // Updated to accept dynamic minimum size
    std::vector<std::pair<double, octomap::point3d>> findFrontierClusters(int min_size) {
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
                    {p.x()+(float)res, p.y(), p.z()}, {p.x()-(float)res, p.y(), p.z()},
                    {p.x(), p.y()+(float)res, p.z()}, {p.x(), p.y()-(float)res, p.z()},
                    {p.x(), p.y(), p.z()+(float)res}, {p.x(), p.y(), p.z()-(float)res}
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

        // Scoring: Size + Progress Alignment Bias
        std::vector<std::pair<double, octomap::point3d>> scored_clusters;
        for (auto& c : clusters) {
            if (c.size() >= (size_t)min_size) {
                octomap::point3d avg(0,0,0);
                for (auto& p : c) avg += p;
                octomap::point3d center = avg * (1.0/c.size());

                // Use graph progress instead of yaw for stable forward bias
                double score = (double)c.size();
                if (graph_.size() >= 2) {
                    octomap::point3d progress_vec = (graph_.back().position - graph_[graph_.size()-2].position).normalized();
                    octomap::point3d cluster_vec = (center - current_pos_).normalized();
                    if (progress_vec.dot(cluster_vec) > 0.4) {
                        score *= HEADING_BIAS; 
                    }
                }

                scored_clusters.push_back({score, center});
            }
        }

        std::sort(scored_clusters.rbegin(), scored_clusters.rend());
        return scored_clusters;
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
    int current_min_size_;
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