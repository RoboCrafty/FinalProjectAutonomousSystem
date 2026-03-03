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
const int ABSOLUTE_MIN_SIZE = 60;           // Absolute minimum cluster size to even consider (for dynamic scaling)
const double CLUSTER_DIST_THRESHOLD = 1.2;  // Distance for grouping frontier voxels
const double TARGET_DISTANCE_OFFSET = 4.0;  // Stop 4m before the cluster center
const double REACHED_THRESHOLD = 1.2;       // Distance to consider setpoint reached
const double FORBIDDEN_RADIUS = 8.0;        // Ignore frontiers near previous graph nodes
const double HEADING_BIAS = 2.0;            // Score multiplier for clusters in front
const double MAX_PLANNING_DIST = 40.0;      // Max distance to attempt a direct flight
const double LOCAL_BOX_SIZE = 50.0;         // Size of the local area to search for frontiers (to save CPU)
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
            // 1. Search for a NEW path during backtrack to resume exploring
            int backoff_threshold = MIN_CLUSTER_SIZE;
            bool found_new_path = false;

            while (backoff_threshold >= (ABSOLUTE_MIN_SIZE)) {
                auto clusters = findFrontierClusters(backoff_threshold);
                for (const auto& pair : clusters) {
                    octomap::point3d center = pair.second;
                    octomap::point3d dir = (center - current_pos_).normalized();
                    octomap::point3d candidate = center - (dir * TARGET_DISTANCE_OFFSET);

                    if (isPathClear(current_pos_, candidate)) {
                        RCLCPP_INFO(this->get_logger(), "New path found during backtrack! Resuming EXPLORING.");
                        state_ = State::EXPLORING;
                        current_min_size_ = backoff_threshold;
                        current_target_ = candidate;
                        publishTarget(current_target_);
                        has_target_ = true;
                        return; // Switch to Exploring and exit
                    }
                }
                backoff_threshold -= 30;
            }

            // 2. Mission Completion Check
            if (graph_.size() <= 1) {
                is_active_ = false;
                RCLCPP_INFO(this->get_logger(), "BACK TO START. Mission Complete.");
                return;
            }

            // 3. Move toward the parent node safely
            int target_id = graph_.back().parent_id;
            octomap::point3d backtrack_pos;
            bool found_parent = false;

            for (const auto& node : graph_) {
                if (node.id == target_id) {
                    backtrack_pos = node.position;
                    found_parent = true;
                    break;
                }
            }

            if (!found_parent) {
                RCLCPP_ERROR(this->get_logger(), "Parent %d not found! Emergency Stop.", target_id);
                is_active_ = false;
                return;
            }

            // If the path to the parent is blocked, step slowly (1.5m) instead of blindly flying into walls.
            if (!isPathClear(current_pos_, backtrack_pos)) {
                RCLCPP_WARN(this->get_logger(), "Backtrack path to Node %d BLOCKED. Stepping slowly...", target_id);
                octomap::point3d safe_dir = (backtrack_pos - current_pos_).normalized();
                current_target_ = current_pos_ + (safe_dir * 1.5); 
            } else {
                current_target_ = backtrack_pos;
            }

            RCLCPP_INFO(this->get_logger(), "BACKTRACKING: Moving toward Parent Node %d at [%.1f, %.1f]", 
                        target_id, current_target_.x(), current_target_.y());

            publishTarget(current_target_);
            has_target_ = true;

            // Only pop the node if we are actually at the ORIGINAL backtrack position,
            // not just at the 1.5m safe-step.
            if ((current_pos_ - backtrack_pos).norm() < REACHED_THRESHOLD) {
                RCLCPP_INFO(this->get_logger(), "Reached Node %d. Popping from graph.", target_id);
                graph_.pop_back();
            }
        }
    }

    bool isPathClear(octomap::point3d start, octomap::point3d end) {
        octomap::point3d direction = (end - start).normalized();
        double dist = (end - start).norm();
        octomap::point3d hit;

        // 1. Check center line
        if (octree_->castRay(start, direction, hit, true, dist)) return false;

        // 2. Check ceiling clearance (+1.0m)
        octomap::point3d start_up(start.x(), start.y(), start.z() + 1.0);
        octomap::point3d end_up(end.x(), end.y(), end.z() + 1.0);
        if (octree_->castRay(start_up, (end_up - start_up).normalized(), hit, true, dist)) return false;

        // 3. Check floor clearance (-1.0m)
        octomap::point3d start_down(start.x(), start.y(), start.z() - 1.0);
        octomap::point3d end_down(end.x(), end.y(), end.z() - 1.0);
        if (octree_->castRay(start_down, (end_down - start_down).normalized(), hit, true, dist)) return false;

        return true; 
    }

    // Updated to accept dynamic minimum size
    std::vector<std::pair<double, octomap::point3d>> findFrontierClusters(int min_size) {
        std::vector<octomap::point3d> frontiers;
        double res = octree_->getResolution();

        // Limit search to a local box to save CPU
        octomap::point3d min_query(current_pos_.x() - LOCAL_BOX_SIZE, current_pos_.y() - LOCAL_BOX_SIZE, current_pos_.z() - LOCAL_BOX_SIZE);
        octomap::point3d max_query(current_pos_.x() + LOCAL_BOX_SIZE, current_pos_.y() + LOCAL_BOX_SIZE, current_pos_.z() + LOCAL_BOX_SIZE);

        for (auto it = octree_->begin_leafs_bbx(min_query, max_query); it != octree_->end_leafs_bbx(); ++it) {
            if (!octree_->isNodeOccupied(*it)) {
                octomap::point3d p = it.getCoordinate();
                
                // IGNORE ENTRANCE/HISTORY
                bool too_close_to_history = false;
                for (const auto& node : graph_) {
                    if ((p - node.position).norm() < FORBIDDEN_RADIUS) {
                        too_close_to_history = true;
                        break;
                    }
                }
                if (too_close_to_history) continue;

                // Frontier check (touches unknown)
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
        n.id = node_count_++; // persistent counter
        n.position = current_pos_;
        if (graph_.empty()) {
            n.parent_id = -1;
        } else {
            // If we are backtracking, connect to the target parent
            if (state_ == State::BACKTRACKING) {
                n.parent_id = graph_.back().parent_id; 
            } else {
                n.parent_id = graph_.back().id;
            }
        }
        graph_.push_back(n);
        RCLCPP_INFO(this->get_logger(), ">>> NODE %d CREATED (Parent: %d) at [%.1f, %.1f]", n.id, n.parent_id, n.position.x(), n.position.y());
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
    int node_count_ = 0;
    bool is_active_;
    bool has_odom_ = false;
    bool has_target_ = false;

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