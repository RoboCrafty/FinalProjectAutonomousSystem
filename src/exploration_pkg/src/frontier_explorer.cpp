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
#include <optional>



// --- TUNABLE PARAMETERS -----------------
const int MIN_CLUSTER_SIZE = 150;           // Minimum number of frontier voxels to consider a valid cluster
const int ABSOLUTE_MIN_SIZE = 60;           // Absolute minimum cluster size to even consider (for dynamic scaling)
const int ABSOLUTE_MIN_SIZE_BACKTRACK = 60; // Even more lenient during backtracking
const double CLUSTER_DIST_THRESHOLD = 1.2;  // Distance for grouping frontier voxels
const double TARGET_DISTANCE_OFFSET = 4.0;  // Stop 4m before the cluster center
const double REACHED_THRESHOLD = 1.2;       // Distance to consider setpoint reached
const double FORBIDDEN_RADIUS = 8.0;        // Ignore frontiers near previous graph nodes
const double HEADING_BIAS = 2.0;            // Score multiplier for clusters in front
const double MAX_PLANNING_DIST = 50.0;      // Max distance to attempt a direct flight
const double LOCAL_BOX_SIZE = 60.0;         // Size of the local area to search for frontiers (to save CPU)
// ----------------------------------------

struct GraphNode {
    int id;
    octomap::point3d position;
    int parent_id = -1;
};

enum class State { EXPLORING, BACKTRACKING };

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer() : Node("frontier_explorer"), state_(State::EXPLORING), is_active_(false), has_target_(false) {
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&FrontierExplorer::odomCallback, this, std::placeholders::_1));

        sub_map_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 1, [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
                octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                if (tree) octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
            });

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { 
                bool was_active = is_active_;
                is_active_ = msg->data;

                if (!is_active_ && was_active && has_odom_) {
                    last_check_pos_ = current_pos_;
                    RCLCPP_INFO(this->get_logger(), "State machine took control. Passive safety monitoring started.");
                }

                if (is_active_ && !was_active && has_odom_) {
                    RCLCPP_INFO(this->get_logger(), "Exploration re-enabled. Anchoring current position.");
                    addNode(); // Store current position as new node on resume
                    
                    if (!entrance_captured_) {
                        cave_entrance_x_ = current_pos_.x();
                        entrance_captured_ = true;
                    }
                }
            });

        pub_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        pub_finished_ = this->create_publisher<std_msgs::msg::Bool>("/exploration_complete", 10);
        
        // Initialize dynamic threshold
        current_min_size_ = MIN_CLUSTER_SIZE;

        RCLCPP_INFO(this->get_logger(), "Frontier Explorer Ready.");
    }

private:

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!has_odom_) {
            current_pos_ = octomap::point3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            last_check_pos_ = current_pos_;
            has_odom_ = true;
            return;
        }

        prev_pos_ = current_pos_;
        current_pos_ = octomap::point3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double r, p; tf2::Matrix3x3(q).getRPY(r, p, current_yaw_);
        has_odom_ = true;

        if (!octree_) return;

        // PASSIVE LOGGING MODE (not actively exploring, just building graph and capturing entrance)
        if (!is_active_) {
            handlePassiveLogging(prev_pos_);
            return; // Exit early: do not publish targets if inactive
        }

        // ACTIVE EXPLORATION MODE
        bool reached = has_target_ && (current_pos_ - current_target_).norm() < REACHED_THRESHOLD;

        if (!has_target_ || reached) {
            if (reached && state_ == State::EXPLORING) {
                RCLCPP_INFO(this->get_logger(), "EXPLORING: Saving Node %d @ [%.1f, %.1f, %.1f]. Parent node: %d.", node_count_, graph_.back().position.x(), graph_.back().position.y(), graph_.back().position.z(), graph_.back().id);
                addNode(); 
            }
            computeNextGoal();
        }
    }



    void computeNextGoal() {
        // EXPLORATION
        if (state_ == State::EXPLORING) {
            if (graph_.empty()) addNode();

            octomap::point3d next_target;
            int size_found;

            if (findBestFrontier(next_target, size_found)) {
                current_target_ = next_target;
                current_min_size_ = std::min(MIN_CLUSTER_SIZE, size_found + 20);
                publishTarget(current_target_);
                has_target_ = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "No frontiers found. Switching to BACKTRACKING.");
                state_ = State::BACKTRACKING;
                computeNextGoal();
            }
        }
        
        // BACKTRACKING WORKFLOW
        else { 
            // Check for mission completion
            if (graph_.size() <= 1) {
                is_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Backtracking complete. Mission finished.");
                auto finish_msg = std_msgs::msg::Bool();
                finish_msg.data = true;
                pub_finished_->publish(finish_msg);
                return;
            }

            int target_id = graph_.back().parent_id;
            octomap::point3d backtrack_pos;

            // Search for parent node position
            for (const auto& node : graph_) {
                if (node.id == target_id) {
                    backtrack_pos = node.position;
                    break;
                }
            }

            // Move to previous node
            double dist_to_parent = (current_pos_ - backtrack_pos).norm();
            
            if (dist_to_parent > REACHED_THRESHOLD) {
                current_target_ = backtrack_pos;
                publishTarget(current_target_);
                has_target_ = true;
            } 
            else {
                // Pop the child node once reached
                RCLCPP_INFO(this->get_logger(), "BACKTRACKING: Reached Node %d. Popping child %d.", target_id, graph_.back().id);
                graph_.pop_back();
                has_target_ = false;

                // Scan for new frontiers immediately upon reaching the node
                octomap::point3d next_target;
                int size_found;
                if (findBestFrontier(next_target, size_found)) {
                    RCLCPP_WARN(this->get_logger(), "New path found during backtrack! Resuming EXPLORING.");
                    state_ = State::EXPLORING;
                    current_target_ = next_target;
                    current_min_size_ = std::min(MIN_CLUSTER_SIZE, size_found + 20);
                    publishTarget(current_target_);
                    has_target_ = true;
                } else {
                    computeNextGoal();
                }
            }
        }
    }



    void handlePassiveLogging(octomap::point3d prev_pos) {
        if (graph_.empty()) return;

        // Check distance since the last time we verified the line-of-sight
        double dist_since_last_check = (current_pos_ - last_check_pos_).norm();

        // Every 5 meters, verify if we can still "see" the last node in the graph
        if (dist_since_last_check >= 5.0) {
            last_check_pos_ = current_pos_; // Update the anchor for the next 5m window

            if (!isPathClear(current_pos_, graph_.back().position)) {
                RCLCPP_WARN(this->get_logger(), "Line-of-sight lost. Dropping connection node.");
                addNode(prev_pos);
            }
        }
    }



    bool isPathClear(octomap::point3d start, octomap::point3d end) {
        octomap::point3d direction = (end - start).normalized();
        double dist = (end - start).norm();
        octomap::point3d hit;

        // Check center line
        if (octree_->castRay(start, direction, hit, true, dist)) return false;
        return true; 
    }



    bool findBestFrontier(octomap::point3d& target_out, int& size_used_out) {
        int limit = (state_ == State::EXPLORING) ? ABSOLUTE_MIN_SIZE : ABSOLUTE_MIN_SIZE_BACKTRACK;
        int search_threshold = MIN_CLUSTER_SIZE;

        while (search_threshold >= limit) {
            auto scored_clusters = findFrontierClusters(search_threshold);
            for (const auto& pair : scored_clusters) {
                octomap::point3d center = pair.second;

                if (center.x() > (cave_entrance_x_ + 2.0)) continue;
                if ((center - current_pos_).norm() > MAX_PLANNING_DIST) continue;

                octomap::point3d dir = (center - current_pos_).normalized();
                octomap::point3d candidate = center - (dir * TARGET_DISTANCE_OFFSET);

                if (isPathClear(current_pos_, candidate)) {
                    target_out = candidate;
                    size_used_out = search_threshold;
                    return true;
                }
            }
            search_threshold -= 30;
        }
        return false;
    }



    // Returns a list of frontier cluster centers sorted by score (size + heading bias)
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

                // Use graph progress for stable forward bias
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

        std::sort(scored_clusters.rbegin(), scored_clusters.rend(),
        [](const std::pair<double, octomap::point3d>& a, const std::pair<double, octomap::point3d>& b) {
            return a.first > b.first; // Sort by score (first element) descending
        });
        return scored_clusters;
    }




    void addNode(std::optional<octomap::point3d> pos = std::nullopt) {
        GraphNode n;
        n.id = node_count_++; // persistent counter
        n.position = pos.value_or(current_pos_);

        if (graph_.empty()) {
            n.parent_id = -1;
        } else {
            // if we are backtracking, connect to the target parent
            if (state_ == State::BACKTRACKING) n.parent_id = graph_.back().parent_id; 
            else n.parent_id = graph_.back().id;
        }
        graph_.push_back(n);
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
    octomap::point3d current_pos_, prev_pos_, last_check_pos_, current_target_;
    double current_yaw_;
    int current_min_size_;
    int node_count_ = 0;
    bool is_active_;
    bool has_odom_ = false;
    bool has_target_ = false;
    double cave_entrance_x_ = 0.0;
    bool entrance_captured_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_finished_;
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}