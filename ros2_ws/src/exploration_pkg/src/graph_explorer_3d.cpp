#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>

using namespace std::chrono_literals;

// --- GRAPH DATA STRUCTURES ---
struct GraphNode {
    int id;
    double x, y, z;
    std::vector<int> neighbors; // IDs of connected nodes
};

struct Frontier {
    double x, y, z;
};

struct Candidate {
    double x, y, z, score;
};

// State Machine for the Drone
enum ExplorerState { EXPLORING, RELOCATING };

class GraphExplorer3D : public rclcpp::Node {
public:
    GraphExplorer3D() : Node("graph_explorer_3d_node"), is_exploring_(false), state_(EXPLORING), next_node_id_(0), start_z_(0.0) {
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10, std::bind(&GraphExplorer3D::enableCb, this, std::placeholders::_1));
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&GraphExplorer3D::odomCb, this, std::placeholders::_1));
        
        sub_map_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&GraphExplorer3D::mapCb, this, std::placeholders::_1));

        pub_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        
        // Back to a safe 500ms so the trajectory generator can actually fly
        timer_ = this->create_wall_timer(500ms, std::bind(&GraphExplorer3D::timerLoop, this));

        RCLCPP_INFO(this->get_logger(), "SIMULATION MAX Explorer Initialized! Ready to map massive caves.");
    }

private:
    void enableCb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !is_exploring_) {
            is_exploring_ = true;
            start_z_ = current_z_; // Altitude Anchor Locked!
            addGraphNode(current_x_, current_y_, current_z_);
            RCLCPP_INFO(this->get_logger(), "EXPLORATION ACTIVATED! Altitude anchored at: %f", start_z_);
        }
    }

    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x; 
        current_y_ = msg->pose.pose.position.y; 
        current_z_ = msg->pose.pose.position.z;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q); double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void mapCb(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
    }

    // --- GRAPH MANAGEMENT ---
    int addGraphNode(double x, double y, double z) {
        GraphNode n;
        n.id = next_node_id_++;
        n.x = x; n.y = y; n.z = z;
        
        if (!graph_.empty()) {
            int closest_id = getClosestGraphNode(x, y, z);
            n.neighbors.push_back(closest_id);
            graph_[closest_id].neighbors.push_back(n.id);
        }
        graph_.push_back(n);
        return n.id;
    }

    int getClosestGraphNode(double x, double y, double z) {
        double min_dist = 999999.0;
        int closest_id = 0;
        for (const auto& node : graph_) {
            double dist = std::sqrt(std::pow(node.x - x, 2) + std::pow(node.y - y, 2) + std::pow(node.z - z, 2));
            if (dist < min_dist) { min_dist = dist; closest_id = node.id; }
        }
        return closest_id;
    }

    // --- DIJKSTRA'S ALGORITHM ---
    std::vector<geometry_msgs::msg::Point> runDijkstra(int start_id, int goal_id) {
        std::unordered_map<int, double> distances;
        std::unordered_map<int, int> previous;
        auto cmp = [&distances](int left, int right) { return distances[left] > distances[right]; };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> queue(cmp);

        for (const auto& node : graph_) distances[node.id] = 999999.0;
        distances[start_id] = 0.0;
        queue.push(start_id);

        while (!queue.empty()) {
            int current = queue.top();
            queue.pop();

            if (current == goal_id) break;

            for (int neighbor : graph_[current].neighbors) {
                double dx = graph_[neighbor].x - graph_[current].x;
                double dy = graph_[neighbor].y - graph_[current].y;
                double dz = graph_[neighbor].z - graph_[current].z;
                double weight = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                double alt = distances[current] + weight;
                if (alt < distances[neighbor]) {
                    distances[neighbor] = alt;
                    previous[neighbor] = current;
                    queue.push(neighbor);
                }
            }
        }

        std::vector<geometry_msgs::msg::Point> path;
        int curr = goal_id;
        while (previous.find(curr) != previous.end()) {
            geometry_msgs::msg::Point p;
            p.x = graph_[curr].x; p.y = graph_[curr].y; p.z = graph_[curr].z;
            path.push_back(p);
            curr = previous[curr];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // --- MAIN LOOP ---
    void timerLoop() {
        if (!is_exploring_ || !octree_) return;

        double dist_to_target = std::sqrt(std::pow(target_x_ - current_x_, 2) + std::pow(target_y_ - current_y_, 2) + std::pow(target_z_ - current_z_, 2));

        if (state_ == RELOCATING) {
            if (dist_to_target < 3.0) {
                if (!relocation_path_.empty()) {
                    auto next_pt = relocation_path_.front();
                    relocation_path_.erase(relocation_path_.begin());
                    publishTarget(next_pt.x, next_pt.y, next_pt.z);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Arrived at Missed Tunnel. Resuming Exploration!");
                    state_ = EXPLORING;
                }
            }
            return;
        }

        // --- EXPLORING STATE ---
        // Stutter Fix: Give it 6 meters of runway since leaps are up to 18m now
        if (dist_to_target > 6.0 && has_target_) return; 

        double best_score = -9999.0;
        double best_x = 0, best_y = 0, best_z = 0;
        
        std::vector<Candidate> candidates;

        std::random_device rd; std::mt19937 gen(rd());
        
        // SIMULATION MODE: Massive vision and massive leaps for a huge cave
        std::uniform_real_distribution<> yaw_dist(-1.2, 1.2); 
        std::uniform_real_distribution<> pitch_dist(-1.0, 1.0);
        std::uniform_real_distribution<> dist_dist(6.0, 16.0); // 18m strides!

        // 200 candidates to utilize your laptop's CPU and thoroughly scan the large volume
        for (int i = 0; i < 200; ++i) {
            double angle_yaw = current_yaw_ + yaw_dist(gen);
            double angle_pitch = pitch_dist(gen);
            double distance = dist_dist(gen);

            double cand_x = current_x_ + distance * std::cos(angle_yaw) * std::cos(angle_pitch);
            double cand_y = current_y_ + distance * std::sin(angle_yaw) * std::cos(angle_pitch);
            double cand_z = current_z_ + distance * std::sin(angle_pitch);

            if (cand_z < -100.0) cand_z = -100.0; 
            if (cand_z > 100.0) cand_z = 100.0;

            double score = scoreCandidate(cand_x, cand_y, cand_z, angle_yaw, distance);

            if (score > 0) {
                candidates.push_back({cand_x, cand_y, cand_z, score});
            }

            if (score > best_score) {
                best_score = score;
                best_x = cand_x; best_y = cand_y; best_z = cand_z;
            }
        }

        if (best_score > 0) {
            int closest_node = getClosestGraphNode(current_x_, current_y_, current_z_);
            double dist_to_graph = std::sqrt(std::pow(graph_[closest_node].x - current_x_, 2) + std::pow(graph_[closest_node].y - current_y_, 2) + std::pow(graph_[closest_node].z - current_z_, 2));
            
            // Drop breadcrumbs every 12 meters since leaps are larger
            if (dist_to_graph > 12.0) {
                addGraphNode(current_x_, current_y_, current_z_);
            }

            // SMART FRONTIER DEDUPLICATION (Scaled for massive 15m intersections)
            std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
                return a.score > b.score;
            });

            for (const auto& cand : candidates) {
                double dist_from_best = std::sqrt(std::pow(cand.x - best_x, 2) + std::pow(cand.y - best_y, 2));
                
                // If it is > 12m away, it's definitely a different tunnel
                if (dist_from_best > 12.0) { 
                    bool already_known = false;
                    for (const auto& f : frontiers_) {
                        // 15m radius deduplication to stop intersection spam
                        if (std::sqrt(std::pow(cand.x - f.x, 2) + std::pow(cand.y - f.y, 2)) < 15.0) {
                            already_known = true; 
                            break;
                        }
                    }
                    if (!already_known) {
                        frontiers_.push_back({cand.x, cand.y, cand.z});
                        RCLCPP_INFO(this->get_logger(), "Spotted a massive cross-tunnel! Logged to memory.");
                        break; 
                    }
                }
            }

            publishTarget(best_x, best_y, best_z);

        } else {
            // --- DEAD END: INITIATE DIJKSTRA RELOCATION ---
            if (!frontiers_.empty()) {
                RCLCPP_WARN(this->get_logger(), "Dead End! Calculating Dijkstra Path to nearest missed tunnel...");
                
                Frontier best_frontier = frontiers_.back();
                frontiers_.pop_back();

                int start_id = getClosestGraphNode(current_x_, current_y_, current_z_);
                int goal_id = getClosestGraphNode(best_frontier.x, best_frontier.y, best_frontier.z);

                relocation_path_ = runDijkstra(start_id, goal_id);
                
                geometry_msgs::msg::Point f_pt; 
                f_pt.x = best_frontier.x; f_pt.y = best_frontier.y; f_pt.z = best_frontier.z;
                relocation_path_.push_back(f_pt);

                state_ = RELOCATING;
            } else {
                RCLCPP_WARN(this->get_logger(), "Map fully explored! Spinning.");
                publishTarget(current_x_ + 1.0 * std::cos(current_yaw_ + 0.78), current_y_ + 1.0 * std::sin(current_yaw_ + 0.78), current_z_);
            }
        }
    }

    double scoreCandidate(double cx, double cy, double cz, double cand_yaw, double distance) {
        if (!octree_) return -99999.0;
        
        octomap::point3d origin(current_x_, current_y_, current_z_);
        octomap::point3d direction(cx - current_x_, cy - current_y_, cz - current_z_);
        octomap::point3d hit_pt;
        
        bool hit = octree_->castRay(origin, direction, hit_pt, false, distance);
        if (hit) {
            octomap::OcTreeNode* hit_node = octree_->search(hit_pt);
            if (hit_node && octree_->isNodeOccupied(hit_node)) {
                return -99999.0; 
            }
        }

        // Safe R at 2.5: Good for massive caves, but allows slipping through doorways
        double safe_r = 4.0; 
        double res = octree_->getResolution(); 
        if (res <= 0.0) res = 0.4; 
        
        for (double dx = -safe_r; dx <= safe_r; dx += res) {
            for (double dy = -safe_r; dy <= safe_r; dy += res) {
                for (double dz = -safe_r; dz <= safe_r; dz += res) {
                    if (dx*dx + dy*dy + dz*dz > safe_r*safe_r) continue; 
                    octomap::OcTreeNode* node = octree_->search(cx + dx, cy + dy, cz + dz);
                    if (node && octree_->isNodeOccupied(node)) return -99999.0; 
                }
            }
        }

        double score = 0.0; 
        // Search R at 25.0: Since the cave is huge, look further out for the darkness
        double search_r = 15.0; 
        for (double dx = -search_r; dx <= search_r; dx += res*2.0) {
            for (double dy = -search_r; dy <= search_r; dy += res*2.0) {
                for (double dz = -search_r; dz <= search_r; dz += res*2.0) {
                    if (dx*dx + dy*dy + dz*dz > search_r*search_r) continue;
                    if (octree_->search(cx + dx, cy + dy, cz + dz) == nullptr) score += 1.0; 
                }
            }
        }

        double yaw_diff = std::abs(cand_yaw - current_yaw_);
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
        
        // Altitude penalty back up to 20 to fight the massive treasure hunt score
        score -= (std::abs(cz - start_z_) * 20.0);

        // Turn penalty
        return score - (std::abs(yaw_diff) * 400.0);
    }

    void publishTarget(double x, double y, double z) {
        target_x_ = x; target_y_ = y; target_z_ = z; has_target_ = true;
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now(); msg.header.frame_id = "world";
        msg.pose.position.x = x; msg.pose.position.y = y; msg.pose.position.z = z;
        tf2::Quaternion q; q.setRPY(0, 0, std::atan2(y - current_y_, x - current_x_));
        msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y(); msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
        pub_target_->publish(msg);
    }

    // Class Variables
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<octomap::OcTree> octree_;
    bool is_exploring_, has_target_;
    ExplorerState state_;
    double current_x_, current_y_, current_z_, current_yaw_;
    double target_x_, target_y_, target_z_, start_z_;

    // Graph Memory
    std::vector<GraphNode> graph_;
    int next_node_id_;
    std::vector<Frontier> frontiers_;
    std::vector<geometry_msgs::msg::Point> relocation_path_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<GraphExplorer3D>()); 
    rclcpp::shutdown(); 
    return 0;
}