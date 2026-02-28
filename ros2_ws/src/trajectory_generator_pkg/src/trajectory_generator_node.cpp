#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
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

        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/next_setpoint", 10, std::bind(&TrajectoryGeneratorNode::goalCallback, this, std::placeholders::_1));

        playback_timer_ = this->create_wall_timer(10ms, std::bind(&TrajectoryGeneratorNode::playbackCallback, this));
        RCLCPP_INFO(this->get_logger(), "Adaptive Orientation Trajectory Generator Ready.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        current_vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
        current_ori_ = msg->pose.pose.orientation;
        has_odom_ = true;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!has_odom_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for Odom feedback...");
            return;
        }

        // Check if the incoming message has a valid orientation set.
        // If all fields are 0, it is uninitialized, so we keep the drone's current pose.
        bool orientation_provided = (std::abs(msg->pose.orientation.x) + 
                                     std::abs(msg->pose.orientation.y) + 
                                     std::abs(msg->pose.orientation.z) + 
                                     std::abs(msg->pose.orientation.w)) > 0.01;

        geometry_msgs::msg::Quaternion target_ori;
        if (orientation_provided) target_ori = msg->pose.orientation;
        else target_ori = current_ori_;

        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
        mav_trajectory_generation::Vertex::Vector vertices;

        // Start point (Current position/velocity)
        mav_trajectory_generation::Vertex start(dimension);
        start.makeStartOrEnd(current_pos_, derivative_to_optimize);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_vel_);
        vertices.push_back(start);

        // End point (Target position)
        mav_trajectory_generation::Vertex end(dimension);
        Eigen::Vector3d goal_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        end.makeStartOrEnd(goal_pos, derivative_to_optimize);
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
        vertices.push_back(end);

        // Optimization
        std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
        mav_trajectory_generation::PolynomialOptimization<10> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        // Sample trajectory and populate points
        std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint> new_points;
        for (double t = 0.0; t <= trajectory.getMaxTime(); t += 0.01) {
            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
            Eigen::VectorXd p = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
            Eigen::VectorXd v = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::VELOCITY);
            Eigen::VectorXd a = trajectory.evaluate(t, mav_trajectory_generation::derivative_order::ACCELERATION);

            geometry_msgs::msg::Transform tf;
            tf.translation.x = p(0); tf.translation.y = p(1); tf.translation.z = p(2);
            
            // Set the rotation (Either from mission or kept from current state)
            tf.rotation = target_ori;

            geometry_msgs::msg::Twist vel_msg, acc_msg;
            vel_msg.linear.x = v(0); vel_msg.linear.y = v(1); vel_msg.linear.z = v(2);
            acc_msg.linear.x = a(0); acc_msg.linear.y = a(1); acc_msg.linear.z = a(2);

            point.transforms.push_back(tf);
            point.velocities.push_back(vel_msg);
            point.accelerations.push_back(acc_msg);
            new_points.push_back(point);
        }

        trajectory_points_ = new_points;
        current_traj_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "Trajectory success! Pts: %zu", trajectory_points_.size());
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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::TimerBase::SharedPtr playback_timer_;
    
    Eigen::Vector3d current_pos_, current_vel_;
    geometry_msgs::msg::Quaternion current_ori_; //stores orientation from Odom
    
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