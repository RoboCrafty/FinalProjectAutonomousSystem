#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class FrontierDetector : public rclcpp::Node {
private:
    struct Point3D {
        double x, y, z;
    };
    
    struct Frontier {
        Point3D coordinates;
        int neighborcount = 0;
        double score = 0;
        bool isReachable = true;
    };

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_subscriber_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr frontier_goal_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double drone_yaw_;
    Point3D curr_drone_position_, cave_entry_point_;
    geometry_msgs::msg::PoseStamped frontier_goal_message_; 
    float octomap_res_;
    bool is_exploring_;
    
    // The "Smooth Flight" & "Perfect Memory" Tuning 
    int neighborcount_threshold_ = 10; 
    double bandwidth_ = 17.0;
    double k_distance_ = 2.0;       // Strong enough to lock onto the closest real tunnel
    double k_neighborcount_ = 0.1;  // Official value
    double k_yaw_ = 10.0;           // Dropped to prevent oscillation
    double distance_limit_ = 1000.0; // Huge memory limit
    int occ_neighbor_threshold_ = 10; // Increased to allow targets near walls
    bool has_odom_ = false;

public:
    FrontierDetector() : Node("frontier_detector_node"), is_exploring_(false) {
        
        octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 1, std::bind(&FrontierDetector::parseOctomap, this, std::placeholders::_1));
            
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&FrontierDetector::currentPositionCb, this, std::placeholders::_1));
            
        state_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10, std::bind(&FrontierDetector::onStateStm, this, std::placeholders::_1));
            
        frontier_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 1);
        
        // Replaced Custom msg with PoseStamped for the RRT planner
        frontier_goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/frontier_goal", 1);
        
        // Timer to publish the goal
        timer_ = this->create_wall_timer(500ms, std::bind(&FrontierDetector::publishGoal, this));

        // Cave entry coordinates
        cave_entry_point_.x = -321.0;
        cave_entry_point_.y = 8.38;
        cave_entry_point_.z = 15.0;

        RCLCPP_INFO(this->get_logger(), "ROS 2 Frontier Detector Initialized!");
    }
    
private:
    void onStateStm(const std_msgs::msg::Bool::SharedPtr msg) {
        is_exploring_ = msg->data;
    }
        
    void currentPositionCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        curr_drone_position_.x = msg->pose.pose.position.x;
        curr_drone_position_.y = msg->pose.pose.position.y;
        curr_drone_position_.z = msg->pose.pose.position.z;
        
        tf2::Quaternion quat(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, drone_yaw_);
        has_odom_ = true;
    }
        
    double euclideanDistance(Point3D p1, Point3D p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double gaussianKernel(double distance) {
        return std::exp(-(distance * distance) / (2 * bandwidth_ * bandwidth_));
    }

    std::vector<Point3D> meanShiftClustering(std::vector<Frontier> points, double convergenceThreshold) {
        std::vector<Point3D> shiftedPoints;
        for(const auto& frontier : points){
            shiftedPoints.push_back(frontier.coordinates);
        }

        bool converged = false;
        int max_iters = 20;
        int iters = 0;

        while (!converged && iters < max_iters) {
            converged = true;
            iters++;

            for (size_t i = 0; i < shiftedPoints.size(); ++i) {
                Point3D originalPoint = shiftedPoints[i];
                Point3D shiftVector = {0.0, 0.0, 0.0};
                double totalWeight = 0.0;

                for (const Frontier& p : points) {
                    double distance = euclideanDistance(originalPoint, p.coordinates);
                    if(distance > bandwidth_ * 3) continue; 
                    
                    double weight = gaussianKernel(distance);
                    shiftVector.x += p.coordinates.x * weight;
                    shiftVector.y += p.coordinates.y * weight;
                    shiftVector.z += p.coordinates.z * weight;
                    totalWeight += weight;
                }

                if (totalWeight > 0) {
                    shiftVector.x /= totalWeight;
                    shiftVector.y /= totalWeight;
                    shiftVector.z /= totalWeight;

                    double distanceToOriginal = euclideanDistance(originalPoint, shiftVector);
                    if (distanceToOriginal > convergenceThreshold) {
                        shiftedPoints[i] = shiftVector;
                        converged = false;
                    }
                }
            }
        }
        return shiftedPoints;
    }
    
    double getScore(Frontier frontier){        
        double yaw_score = std::abs(std::atan2(frontier.coordinates.y - curr_drone_position_.y, 
                                               frontier.coordinates.x - curr_drone_position_.x) - drone_yaw_);
        if(yaw_score > M_PI){
            yaw_score = (2 * M_PI) - yaw_score;
        }
        return -k_distance_ * euclideanDistance(frontier.coordinates, curr_drone_position_) 
               + k_neighborcount_ * (double)frontier.neighborcount 
               - k_yaw_ * yaw_score;
    }
    
    void publishGoal() {
        if(!is_exploring_ || !has_odom_){
            return;
        }
        if(std::abs(frontier_goal_message_.pose.position.x) > 0.1) {
            frontier_goal_message_.header.stamp = this->now();
            frontier_goal_publisher_->publish(frontier_goal_message_);
        }
    }
    
    void selectFrontier(std::vector<Frontier> points_sorted){
        if(!points_sorted.empty()){
            Frontier best_frontier = points_sorted[0];
            best_frontier.score = getScore(best_frontier);         
            
            for(auto& frontier : points_sorted){
                frontier.score = getScore(frontier);               
                if(frontier.score > best_frontier.score){
                    best_frontier = frontier;                   
                }
            }
            
            if(best_frontier.isReachable && euclideanDistance(curr_drone_position_, best_frontier.coordinates) < distance_limit_){
                frontier_goal_message_.header.frame_id = "world";
                frontier_goal_message_.pose.position.x = best_frontier.coordinates.x;
                frontier_goal_message_.pose.position.y = best_frontier.coordinates.y;
                frontier_goal_message_.pose.position.z = best_frontier.coordinates.z;
                
                double goal_yaw = std::atan2(best_frontier.coordinates.y - curr_drone_position_.y, 
                                             best_frontier.coordinates.x - curr_drone_position_.x);
                tf2::Quaternion q;
                q.setRPY(0, 0, goal_yaw);
                frontier_goal_message_.pose.orientation.x = q.getX();
                frontier_goal_message_.pose.orientation.y = q.getY();
                frontier_goal_message_.pose.orientation.z = q.getZ();
                frontier_goal_message_.pose.orientation.w = q.getW();
                
                RCLCPP_INFO(this->get_logger(), "New Frontier Selected: X:%.1f Y:%.1f", best_frontier.coordinates.x, best_frontier.coordinates.y);
            }
        } else {
            // The cave is empty! Wipe the goal so the State Machine knows to LAND.
            frontier_goal_message_.pose.position.x = 0.0;
            frontier_goal_message_.pose.position.y = 0.0;
            frontier_goal_message_.pose.position.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "No frontiers left! Cave fully explored.");
        }
    }
    
    void publishMarkers(std::vector<Frontier> points_sorted){
        visualization_msgs::msg::MarkerArray frontierMarkers;
        
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = "frontier";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        frontierMarkers.markers.push_back(clear_marker);
            
        for(size_t i = 0; i < points_sorted.size(); i++){
            visualization_msgs::msg::Marker new_frontier_marker;
            new_frontier_marker.header.frame_id = "world";
            new_frontier_marker.header.stamp = this->now();
            new_frontier_marker.ns = "frontier";
            new_frontier_marker.id = i + 1;
            new_frontier_marker.type = visualization_msgs::msg::Marker::CUBE;
            
            new_frontier_marker.pose.position.x = points_sorted[i].coordinates.x;
            new_frontier_marker.pose.position.y = points_sorted[i].coordinates.y;
            new_frontier_marker.pose.position.z = points_sorted[i].coordinates.z;
            new_frontier_marker.pose.orientation.w = 1.0;
            
            new_frontier_marker.scale.x = octomap_res_;
            new_frontier_marker.scale.y = octomap_res_;
            new_frontier_marker.scale.z = octomap_res_;
            
            new_frontier_marker.color.r = 1.0f;
            new_frontier_marker.color.a = 0.8;
            frontierMarkers.markers.push_back(new_frontier_marker);
        }
        frontier_publisher_->publish(frontierMarkers);
    }

    std::vector<Frontier> sortFrontiers(std::vector<Point3D> frontiers_clustered, octomap::OcTree* octree){
        std::vector<Frontier> frontiers_sorted;
        for(const auto& f1 : frontiers_clustered){
            Frontier frontier;
            frontier.coordinates = f1;
            frontier.neighborcount = 0;
            
            for(const auto& f2 : frontiers_clustered){
                if(euclideanDistance(f1, f2) < octomap_res_){
                    frontier.neighborcount++;
                }
            }
            
            bool addflag = true;
            for(const auto& f3 : frontiers_sorted){
                if(euclideanDistance(f3.coordinates, frontier.coordinates) < octomap_res_){
                    addflag = false;
                }
            }
            
            int nbscore = 0;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx != 0 || dy != 0 || dz != 0){
                            octomap::point3d point(frontier.coordinates.x + octomap_res_*dx, 
                                                   frontier.coordinates.y + octomap_res_*dy, 
                                                   frontier.coordinates.z + octomap_res_*dz);
                            octomap::OcTreeNode* result = octree->search(point);
                            if (result && octree->isNodeOccupied(result)) {
                                nbscore++;
                            }   
                        }
                     }
                }
            }
            
            int entry_tol = 25;
            bool add_entry_frontier = !(curr_drone_position_.x < cave_entry_point_.x - entry_tol && 
                                        frontier.coordinates.x > cave_entry_point_.x - entry_tol);
            
            // Calculate distance to the drone and ignore anything closer than 10 meters (Ghost Filter)
            double dist_to_drone = euclideanDistance(curr_drone_position_, frontier.coordinates);
            bool is_not_ghost = dist_to_drone > 10.0; 
            
            if(addflag && frontier.neighborcount > neighborcount_threshold_ && add_entry_frontier && nbscore < occ_neighbor_threshold_ && is_not_ghost){
                frontiers_sorted.push_back(frontier);
            }
        }
        return frontiers_sorted;
    }
    
    void parseOctomap(const octomap_msgs::msg::Octomap::SharedPtr octomap_msg){ 
        if(!is_exploring_ || !has_odom_) return;
        
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*octomap_msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        if (!octree) return;

        octomap_res_ = octree->getResolution();
        std::vector<Frontier> frontiers;
        
        for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); ++it){
            if (!octree->isNodeOccupied(*it)) {
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            if (dx != 0 || dy != 0 || dz != 0){
                                octomap::OcTreeKey neighborKey(it.getKey().k[0] + dx, it.getKey().k[1] + dy, it.getKey().k[2] + dz);
                                if (octree->search(neighborKey) == nullptr) {
                                    Frontier frontierPoint;
                                    frontierPoint.coordinates.x = it.getX();
                                    frontierPoint.coordinates.y = it.getY();
                                    frontierPoint.coordinates.z = it.getZ();
                                    frontiers.push_back(frontierPoint);
                                }
                            }
                        }
                    }
                }
            }
        }
        
        if(!frontiers.empty()) {
            std::vector<Point3D> frontierpoints_clustered = meanShiftClustering(frontiers, 1.0);
            std::vector<Frontier> frontierpoints_sorted = sortFrontiers(frontierpoints_clustered, octree);
            selectFrontier(frontierpoints_sorted);
            publishMarkers(frontierpoints_sorted);
        }
        
        delete octree;
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierDetector>());
    rclcpp::shutdown();
    return 0;
}