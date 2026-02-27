#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_trajectory_generation/timing.h>

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator_node"), 
        current_pose_(Eigen::Affine3d::Identity()), 
        current_velocity_(Eigen::Vector3d::Zero()),
        has_odom_(false) // Prevent generating path before knowing where drone is
    {
        // Force valid default parameters if not provided in launch file
        this->declare_parameter("max_v", 2.0); // Increased slightly for safety
        this->declare_parameter("max_a", 1.0); // Increased slightly for safety
        max_v_ = this->get_parameter("max_v").as_double();
        max_a_ = this->get_parameter("max_a").as_double();

        if (max_v_ <= 0.0 || max_a_ <= 0.0) {
            max_v_ = 2.0;
            max_a_ = 1.0;
            RCLCPP_WARN(this->get_logger(), "Invalid velocity/accel limits. Resetting to V:2.0, A:1.0");
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory Generator Initialized. V: %f, A: %f", max_v_, max_a_);

        // The controller listens to this topic
        pub_trajectory_ = this->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>("/command/trajectory", 10);
        
        // Listen to Odometry to know current position
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&TrajectoryGeneratorNode::odomCallback, this, std::placeholders::_1));

        // Listen to Mission Control (Transient Local to match your mission control)
        rclcpp::QoS path_qos(10);
        path_qos.transient_local();
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", path_qos, std::bind(&TrajectoryGeneratorNode::pathCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr pub_trajectory_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    double max_v_, max_a_;
    bool has_odom_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        Eigen::Translation3d translation(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        Eigen::Quaterniond rotation(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        current_pose_ = translation * rotation;
        current_velocity_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
        has_odom_ = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!has_odom_) {
            RCLCPP_WARN(this->get_logger(), "Received path, but waiting for odometry. Cannot plan from unknown position.");
            return;
        }

        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path is empty.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints. Generating trajectory...", msg->poses.size());

        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
        mav_trajectory_generation::Vertex::Vector vertices;

        // 1. START POINT (Drone's true current position)
        mav_trajectory_generation::Vertex start(dimension);
        start.makeStartOrEnd(current_pose_.translation(), derivative_to_optimize);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_velocity_);
        vertices.push_back(start);

        // 2. MIDDLE WAYPOINTS (Only if the path has more than 1 waypoint)
        if (msg->poses.size() > 1) {
            for (size_t i = 0; i < msg->poses.size() - 1; i++) {
                mav_trajectory_generation::Vertex middle(dimension);
                Eigen::Vector3d pos(msg->poses[i].pose.position.x, 
                                    msg->poses[i].pose.position.y, 
                                    msg->poses[i].pose.position.z);
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
                vertices.push_back(middle);
            }
        }

        // 3. END POINT
        mav_trajectory_generation::Vertex end(dimension);
        Eigen::Vector3d goal_pos(msg->poses.back().pose.position.x, 
                                 msg->poses.back().pose.position.y, 
                                 msg->poses.back().pose.position.z);
        end.makeStartOrEnd(goal_pos, derivative_to_optimize);
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
        vertices.push_back(end);

        // 4. MATH ENGINE (Linear optimization first to prevent stack smashing!)
        std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
        
        // We use Nonlinear Optimization, but if there are only 2 vertices (Start & End), 
        // we must ensure it doesn't crash on constraints.
        mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(dimension, mav_trajectory_generation::NonlinearOptimizationParameters());
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
        
        try {
            opt.optimize();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Optimizer crashed: %s", e.what());
            return;
        }

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        // 5. PUBLISH
        mav_planning_msgs::msg::PolynomialTrajectory4D traj_msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &traj_msg);
        traj_msg.header.frame_id = "world";
        pub_trajectory_->publish(traj_msg);
        
        RCLCPP_INFO(this->get_logger(), "Trajectory sent to controller successfully!");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
