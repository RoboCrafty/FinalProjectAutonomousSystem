#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <Eigen/Dense>
#include <vector>

// Use the Linear optimization for stability
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/timing.h>

using namespace std::chrono_literals;

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator_node"), has_odom_(false), current_traj_idx_(0) {
        this->declare_parameter("max_v", 15.0); 
        this->declare_parameter("max_a", 5.0); 
        max_v_ = this->get_parameter("max_v").as_double();
        max_a_ = this->get_parameter("max_a").as_double();

        pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/command/trajectory", 10);
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, std::bind(&TrajectoryGeneratorNode::odomCallback, this, std::placeholders::_1));

        // UPGRADE: Now listening to the full RRT Path instead of a single setpoint
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/rrt_path", 10, std::bind(&TrajectoryGeneratorNode::pathCallback, this, std::placeholders::_1));

        playback_timer_ = this->create_wall_timer(10ms, std::bind(&TrajectoryGeneratorNode::playbackCallback, this));
        RCLCPP_INFO(this->get_logger(), "Fluid Multi-Waypoint Trajectory Generator Ready.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        current_vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
        current_ori_ = msg->pose.pose.orientation;
        has_odom_ = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!has_odom_ || msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Waiting for Odom feedback or received empty path...");
            return;
        }

        // Grab the orientation from the final goal in the path
        auto final_pose = msg->poses.back().pose;
        bool orientation_provided = (std::abs(final_pose.orientation.x) + 
                                     std::abs(final_pose.orientation.y) + 
                                     std::abs(final_pose.orientation.z) + 
                                     std::abs(final_pose.orientation.w)) > 0.01;

        geometry_msgs::msg::Quaternion target_ori;
        if (orientation_provided) target_ori = final_pose.orientation;
        else target_ori = current_ori_;

        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
        mav_trajectory_generation::Vertex::Vector vertices;

        // 1. START POINT: Lock to current position and current velocity
        mav_trajectory_generation::Vertex start(dimension);
        start.makeStartOrEnd(current_pos_, derivative_to_optimize);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_vel_);
        vertices.push_back(start);

        // 2. INTERMEDIATE POINTS (Swoop through them without stopping)
        Eigen::Vector3d last_added_pos = current_pos_;
        
        for (size_t i = 0; i < msg->poses.size() - 1; ++i) {
            Eigen::Vector3d pos(msg->poses[i].pose.position.x, 
                                msg->poses[i].pose.position.y, 
                                msg->poses[i].pose.position.z);
            
            // Safety filter: Ignore waypoints that are closer than 0.5m to prevent math singularities
            if ((pos - last_added_pos).norm() > 0.5) {
                mav_trajectory_generation::Vertex intermediate(dimension);
                intermediate.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
                vertices.push_back(intermediate);
                last_added_pos = pos;
            }
        }

        // 3. END POINT: Target position, come to a complete stop (Zero Velocity)
        mav_trajectory_generation::Vertex end(dimension);
        Eigen::Vector3d goal_pos(final_pose.position.x, final_pose.position.y, final_pose.position.z);
        
        // Ensure the final point is also far enough from the last added intermediate point
        if ((goal_pos - last_added_pos).norm() > 0.1 || vertices.size() == 1) {
            end.makeStartOrEnd(goal_pos, derivative_to_optimize);
            end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
            vertices.push_back(end);
        }

        // Run the polynomial optimization
        std::vector<double> segment_times;
        try {
            segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory timing failed: %s", e.what());
            return;
        }

        mav_trajectory_generation::PolynomialOptimization<10> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        // Sample trajectory and populate points at 100Hz (0.01s)
        std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint> new_points;
        for (double t = 0.0; t <= trajectory.getMaxTime(); t += 0.01) {
            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
            Eigen::VectorXd p = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
            Eigen::VectorXd v = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::VELOCITY);
            Eigen::VectorXd a = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::ACCELERATION);

            geometry_msgs::msg::Transform tf;
            tf.translation.x = p(0); tf.translation.y = p(1); tf.translation.z = p(2);
            tf.rotation = target_ori;

            geometry_msgs::msg::Twist vel_msg, acc_msg;
            vel_msg.linear.x = v(0); vel_msg.linear.y = v(1); vel_msg.linear.z = v(2);
            acc_msg.linear.x = a(0); acc_msg.linear.y = a(1); acc_msg.linear.z = a(2);

            point.transforms.push_back(tf);
            point.velocities.push_back(vel_msg);
            point.accelerations.push_back(acc_msg);
            new_points.push_back(point);
        }

        // Safely overwrite the active playback path
        trajectory_points_ = new_points;
        current_traj_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "Fluid Trajectory generated! Vertices: %zu, Pts: %zu", vertices.size(), trajectory_points_.size());
    }

    void playbackCallback() {
        if (has_odom_ && !trajectory_points_.empty() && current_traj_idx_ < trajectory_points_.size()) {
            trajectory_msgs::msg::MultiDOFJointTrajectory msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "world";
            msg.points.push_back(trajectory_points_[current_traj_idx_++]);
            pub_trajectory_->publish(msg);
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr pub_trajectory_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::TimerBase::SharedPtr playback_timer_;
    
    Eigen::Vector3d current_pos_, current_vel_;
    geometry_msgs::msg::Quaternion current_ori_;
    
    double max_v_, max_a_;
    bool has_odom_;
    std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint> trajectory_points_;
    size_t current_traj_idx_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}