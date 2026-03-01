#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <queue>
#include <unordered_set>
#include <vector>
#include <memory>
#include <algorithm>

// MAV Trajectory Generation Includes
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>

// ---------------------------------------------------------
// A* Node Structure
// ---------------------------------------------------------
struct AStarNode {
    octomap::point3d position;
    double g_cost; // Distance from start
    double h_cost; // Estimated distance to goal
    std::shared_ptr<AStarNode> parent;

    AStarNode(octomap::point3d pos, double g, double h, std::shared_ptr<AStarNode> p = nullptr)
        : position(pos), g_cost(g), h_cost(h), parent(p) {}

    double f_cost() const { return g_cost + h_cost; }
};

struct CompareNode {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
        return a->f_cost() > b->f_cost();
    }
};

// ---------------------------------------------------------
// Main Path Planner Class
// ---------------------------------------------------------
class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner_node") {
        
        // 1. Publishers
        trajectory_pub_ = this->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
            "/command/trajectory", 10);
            
        // Debug publisher to visualize the raw A* path in RViz
        raw_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/raw_astar_path", 10);

        // 2. Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10,
            std::bind(&PathPlannerNode::odomCallback, this, std::placeholders::_1));

        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10,
            std::bind(&PathPlannerNode::octomapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Path Planner + Trajectory Generator Initialized.");
    }

private:
    std::shared_ptr<octomap::OcTree> octree_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_twist_;
    bool has_pose_ = false;
    bool is_planning_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;

    // ---------------------------------------------------------
    // Callbacks
    // ---------------------------------------------------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        current_twist_ = msg->twist.twist;
        has_pose_ = true;
    }

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        if (!has_pose_ || is_planning_) return; 

        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree));
            
            // Only trigger planning once for now (e.g. to fly 5 meters forward)
            is_planning_ = true;
            planPathAndFly();
        }
    }

    // ---------------------------------------------------------
    // Collision Checking
    // ---------------------------------------------------------
    bool isStateValid(const octomap::point3d& point) {
        if (!octree_) return false;

        double safe_radius = 0.4; // 40cm safety bubble around drone
        
        std::vector<octomap::point3d> points_to_check = {
            point,
            octomap::point3d(point.x() + safe_radius, point.y(), point.z()),
            octomap::point3d(point.x() - safe_radius, point.y(), point.z()),
            octomap::point3d(point.x(), point.y() + safe_radius, point.z()),
            octomap::point3d(point.x(), point.y() - safe_radius, point.z()),
            octomap::point3d(point.x(), point.y(), point.z() + safe_radius),
            octomap::point3d(point.x(), point.y(), point.z() - safe_radius)
        };

        for (const auto& pt : points_to_check) {
            octomap::OcTreeNode* result = octree_->search(pt);
            if (result == nullptr) {
                return false; // Unknown space is treated as obstacle for safety
            }
            if (octree_->isNodeOccupied(result)) {
                return false; // Collision!
            }
        }
        return true; 
    }

    // ---------------------------------------------------------
    // A* Graph Search
    // ---------------------------------------------------------
    void planPathAndFly() {
        RCLCPP_INFO(this->get_logger(), "Starting A* Search...");

        octomap::point3d start(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
        // FOR TESTING: Goal is 5 meters along the X axis
        octomap::point3d goal(start.x() - 5.0, start.y(), start.z()); 

        std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareNode> open_list;
        std::unordered_set<std::string> closed_list; 

        auto start_node = std::make_shared<AStarNode>(start, 0.0, start.distance(goal));
        open_list.push(start_node);

        double step = 0.2; // Step size for search (should roughly match octomap resolution)

        std::shared_ptr<AStarNode> final_node = nullptr;

        while (!open_list.empty()) {
            auto current_node = open_list.top();
            open_list.pop();

            if (current_node->position.distance(goal) < step) {
                final_node = current_node;
                break;
            }

            // Create string key (rounded to 1 decimal to avoid float math issues in hashing)
            char key_buffer[50];
            snprintf(key_buffer, sizeof(key_buffer), "%.1f,%.1f,%.1f", 
                     current_node->position.x(), current_node->position.y(), current_node->position.z());
            std::string node_key(key_buffer);

            if (closed_list.find(node_key) != closed_list.end()) continue;
            closed_list.insert(node_key);

            // Generate 26 Neighbors
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) continue; 

                        octomap::point3d neighbor_pos(
                            current_node->position.x() + (dx * step),
                            current_node->position.y() + (dy * step),
                            current_node->position.z() + (dz * step)
                        );

                        if (!isStateValid(neighbor_pos)) continue;

                        double movement_cost = current_node->position.distance(neighbor_pos);
                        double new_g = current_node->g_cost + movement_cost;
                        double new_h = neighbor_pos.distance(goal);

                        open_list.push(std::make_shared<AStarNode>(neighbor_pos, new_g, new_h, current_node));
                    }
                }
            }
        }

        if (final_node == nullptr) {
            RCLCPP_WARN(this->get_logger(), "A* Failed: No path found to goal!");
            is_planning_ = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "A* Path Found! Reconstructing...");

        // Reconstruct Path
        std::vector<octomap::point3d> raw_path;
        auto trace_node = final_node;
        while (trace_node != nullptr) {
            raw_path.push_back(trace_node->position);
            trace_node = trace_node->parent;
        }
        std::reverse(raw_path.begin(), raw_path.end());

        publishRawPath(raw_path);
        generateSmoothTrajectory(raw_path);
    }

    // ---------------------------------------------------------
    // Trajectory Generation (Polynomial Optimization)
    // ---------------------------------------------------------
    void generateSmoothTrajectory(const std::vector<octomap::point3d>& path) {
        RCLCPP_INFO(this->get_logger(), "Smoothing trajectory...");
        
        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
        mav_trajectory_generation::Vertex::Vector vertices;

        // 1. Add Start Point
        mav_trajectory_generation::Vertex start(dimension);
        start.makeStartOrEnd(Eigen::Vector3d(path.front().x(), path.front().y(), path.front().z()), derivative_to_optimize);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 
                            Eigen::Vector3d(current_twist_.linear.x, current_twist_.linear.y, current_twist_.linear.z));
        vertices.push_back(start);

        // 2. Add Intermediate Waypoints (Subsample to avoid overloading optimizer)
        // Taking every 4th point of the A* path
        int subsample_rate = 4;
        for (size_t i = subsample_rate; i < path.size() - subsample_rate; i += subsample_rate) {
            mav_trajectory_generation::Vertex middle(dimension);
            middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, 
                                 Eigen::Vector3d(path[i].x(), path[i].y(), path[i].z()));
            vertices.push_back(middle);
        }

        // 3. Add Goal Point
        mav_trajectory_generation::Vertex end(dimension);
        end.makeStartOrEnd(Eigen::Vector3d(path.back().x(), path.back().y(), path.back().z()), derivative_to_optimize);
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
        vertices.push_back(end);

        // 4. Optimize
        double max_v = 1.0;
        double max_a = 0.3;
        std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v, max_a);
        
        mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(dimension, mav_trajectory_generation::NonlinearOptimizationParameters());
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a);
        
        opt.optimize();

        // 5. Publish to Controller
        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);
        
        mav_planning_msgs::msg::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
        msg.header.frame_id = "world";
        trajectory_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Trajectory Published to Controller!");
        
        // Reset planning flag so it can plan a new path later
        // is_planning_ = false; 
    }

    // Helper to view the A* output in RViz before smoothing
    void publishRawPath(const std::vector<octomap::point3d>& path) {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "world";
        msg.header.stamp = this->now();

        for (const auto& pt : path) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = pt.x();
            p.pose.position.y = pt.y();
            p.pose.position.z = pt.z();
            msg.poses.push_back(p);
        }
        raw_path_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
