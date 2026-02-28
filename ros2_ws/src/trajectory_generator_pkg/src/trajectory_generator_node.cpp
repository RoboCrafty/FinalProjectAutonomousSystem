#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>

using namespace std::chrono_literals;

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator_node"), 
        current_pose_(Eigen::Affine3d::Identity()), 
        current_velocity_(Eigen::Vector3d::Zero()),
        has_odom_(false),
        current_traj_idx_(0)
    {
        this->declare_parameter("max_v", 3.0); 
        this->declare_parameter("max_a", 2.0); 
        max_v_ = this->get_parameter("max_v").as_double();
        max_a_ = this->get_parameter("max_a").as_double();

        pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/command/trajectory", 10);
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&TrajectoryGeneratorNode::odomCallback, this, std::placeholders::_1));

        rclcpp::QoS path_qos(10);
        path_qos.transient_local();
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", path_qos, std::bind(&TrajectoryGeneratorNode::pathCallback, this, std::placeholders::_1));

        // NEW: A timer that acts as a "feeder" to playback the trajectory to the controller at 50Hz
        playback_timer_ = this->create_wall_timer(20ms, std::bind(&TrajectoryGeneratorNode::playbackCallback, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory Generator Initialized. Feeder ready.");
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr pub_trajectory_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::TimerBase::SharedPtr playback_timer_;

    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    double max_v_, max_a_;
    bool has_odom_;
    
    // Storage for our breadcrumbs
    std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint> trajectory_points_;
    size_t current_traj_idx_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        Eigen::Translation3d translation(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Quaterniond rotation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        current_pose_ = translation * rotation;
        current_velocity_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
        has_odom_ = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!has_odom_ || msg->poses.empty()) return;

        RCLCPP_INFO(this->get_logger(), "Calculating physics curve for %zu waypoints...", msg->poses.size());

        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
        mav_trajectory_generation::Vertex::Vector vertices;

        mav_trajectory_generation::Vertex start(dimension);
        start.makeStartOrEnd(current_pose_.translation(), derivative_to_optimize);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_velocity_);
        vertices.push_back(start);

        if (msg->poses.size() > 1) {
            for (size_t i = 0; i < msg->poses.size() - 1; i++) {
                mav_trajectory_generation::Vertex middle(dimension);
                Eigen::Vector3d pos(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
                vertices.push_back(middle);
            }
        }

        mav_trajectory_generation::Vertex end(dimension);
        Eigen::Vector3d goal_pos(msg->poses.back().pose.position.x, msg->poses.back().pose.position.y, msg->poses.back().pose.position.z);
        end.makeStartOrEnd(goal_pos, derivative_to_optimize);
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
        vertices.push_back(end);

        std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
        
        mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(dimension, mav_trajectory_generation::NonlinearOptimizationParameters());
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
        opt.optimize();

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        // CLEAR OLD BREADCRUMBS
        trajectory_points_.clear();

        double dt = 0.02; // 50Hz
        for (double t = 0.0; t <= trajectory.getMaxTime(); t += dt) {
            Eigen::VectorXd pos = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
            Eigen::VectorXd vel = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::VELOCITY);
            Eigen::VectorXd acc = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::ACCELERATION);

            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
            geometry_msgs::msg::Transform transform;
            transform.translation.x = pos(0); transform.translation.y = pos(1); transform.translation.z = pos(2);
            transform.rotation.w = 1.0; 
            
            geometry_msgs::msg::Twist velocity, acceleration;
            velocity.linear.x = vel(0); velocity.linear.y = vel(1); velocity.linear.z = vel(2);
            acceleration.linear.x = acc(0); acceleration.linear.y = acc(1); acceleration.linear.z = acc(2);

            point.transforms.push_back(transform);
            point.velocities.push_back(velocity);
            point.accelerations.push_back(acceleration);
            trajectory_points_.push_back(point);
        }

        // START PLAYBACK
        current_traj_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "Curve mapped! Feeding %zu breadcrumbs to the Controller...", trajectory_points_.size());
    }

    void playbackCallback() {
        // If we have points left to feed, send exactly ONE to the controller
        if (current_traj_idx_ < trajectory_points_.size()) {
            trajectory_msgs::msg::MultiDOFJointTrajectory msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "world";
            
            // Only push the single current point!
            msg.points.push_back(trajectory_points_[current_traj_idx_]);
            pub_trajectory_->publish(msg);
            
            current_traj_idx_++;
            
            if (current_traj_idx_ == trajectory_points_.size()) {
                RCLCPP_INFO(this->get_logger(), "Final breadcrumb reached! Hovering at destination.");
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
