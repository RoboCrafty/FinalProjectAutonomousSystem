#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/ScopedState.h>
#include <cmath>

using namespace std::chrono_literals;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class OMPLPathPlanner : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr start_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_subscriber_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<octomap::OcTree> map_;
    geometry_msgs::msg::PoseStamped start_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    
    bool start_received_ = false;
    bool goal_received_ = false;
    bool is_exploring_ = false;

    // Parameters tuned 
    double map_resolution_ = 4.5;
    double step_size_factor_ = 2.0; // RRT branch length
    double bias_ = 0.2; // Pull toward goal
    double timeout_ = 3.0; // 
public:
    OMPLPathPlanner() : Node("rrt_planner_node") {
        
        start_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&OMPLPathPlanner::startPoseCallback, this, std::placeholders::_1));
            
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/frontier_goal", 1, std::bind(&OMPLPathPlanner::goalPoseCallback, this, std::placeholders::_1));
            
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 1, std::bind(&OMPLPathPlanner::octomapCallback, this, std::placeholders::_1));
            
        state_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10, std::bind(&OMPLPathPlanner::onStateStm, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rrt_path", 1);

        // Run planner at 2Hz (every 500ms)
        timer_ = this->create_wall_timer(3000ms, std::bind(&OMPLPathPlanner::attemptPlanning, this));
        
        RCLCPP_INFO(this->get_logger(), "ROS 2 OMPL Path Planner initialized.");
    }

private:
    void onStateStm(const std_msgs::msg::Bool::SharedPtr msg) {
        is_exploring_ = msg->data;
    }

    void startPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        start_pose_.pose = msg->pose.pose;
        start_received_ = true;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = *msg;
        goal_received_ = true;
    }

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            map_.reset(dynamic_cast<octomap::OcTree*>(tree));
            map_resolution_ = map_->getResolution();
        }
    }

    bool isStateValid(const ob::State* state) {
        if (!map_) return false;

        const auto* state3D = state->as<ob::RealVectorStateSpace::StateType>();
        double x = state3D->values[0];
        double y = state3D->values[1];
        double z = state3D->values[2];

        octomap::OcTreeNode* node = map_->search(x, y, z);
        
        // THE FIX: If the node is in the dark void (nullptr), treat it as FREE space!
        // This allows the RRT tree to connect to targets just outside the 40m vision limit.
        if (node == nullptr) return true; 

        // THE OFFICIAL REPORT LOGIC: Reject only if it is a known solid rock (Probability > 0.7)
        return node->getOccupancy() <= 0.7; 
    }

    void attemptPlanning() {
        if (!start_received_ || !goal_received_ || !map_ || !is_exploring_) return;

        auto space = std::make_shared<ob::RealVectorStateSpace>(3);
        ob::RealVectorBounds bounds(3);

        double x_min, y_min, z_min, x_max, y_max, z_max;
        map_->getMetricMin(x_min, y_min, z_min);
        map_->getMetricMax(x_max, y_max, z_max);
        
        // Add a buffer to bounds to allow flying near walls
        bounds.setLow(0, x_min - 50.0); bounds.setHigh(0, x_max + 50.0);
        bounds.setLow(1, y_min - 50.0); bounds.setHigh(1, y_max + 50.0);
        bounds.setLow(2, z_min - 50.0); bounds.setHigh(2, z_max + 50.0);
        space->setBounds(bounds);

        // Fast Straight-Line Raycast Check (No need for heavy RRT if path is clear)
        octomap::point3d p1(start_pose_.pose.position.x, start_pose_.pose.position.y, start_pose_.pose.position.z);
        octomap::point3d p2(goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);
        octomap::point3d hit;
        double distance = std::sqrt(std::pow(p1.x()-p2.x(),2) + std::pow(p1.y()-p2.y(),2) + std::pow(p1.z()-p2.z(),2));
        
        if (!map_->castRay(p1, p2 - p1, hit, false, distance)) {
            // Path is clear! Send straight line path.
            nav_msgs::msg::Path ros_path;
            ros_path.header.frame_id = "world";
            ros_path.header.stamp = this->now();
            ros_path.poses.push_back(start_pose_);
            ros_path.poses.push_back(goal_pose_);
            path_pub_->publish(ros_path);
            return;
        }

        // Obstacle detected! Fallback to RRT* to weave around it.
        og::SimpleSetup ss(space);
        ss.setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

        ob::ScopedState<> start(space), goal(space);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = p1.x();
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = p1.y();
        start->as<ob::RealVectorStateSpace::StateType>()->values[2] = p1.z();

        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = p2.x();
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = p2.y();
        goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = p2.z();

        ss.setStartAndGoalStates(start, goal);        

        auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
        planner->setRange(step_size_factor_ * map_resolution_);
        planner->setGoalBias(bias_);
        ss.setPlanner(planner);

        ob::PlannerStatus solved = ss.solve(timeout_);

        if (solved) {
            og::PathGeometric path = ss.getSolutionPath();
            nav_msgs::msg::Path ros_path;
            ros_path.header.frame_id = "world";
            ros_path.header.stamp = this->now();

            for (auto state : path.getStates()) {
                const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = s->values[0];
                pose.pose.position.y = s->values[1];
                pose.pose.position.z = s->values[2];
                // Maintain the target yaw through the curve
                pose.pose.orientation = goal_pose_.pose.orientation; 
                ros_path.poses.push_back(pose);
            }
            path_pub_->publish(ros_path);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OMPLPathPlanner>());
    rclcpp::shutdown();
    return 0;
}