#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <memory>
#include <cmath>

using Eigen::Vector3d;


struct ERRTNode {
    Vector3d position;
    int parent;
    double cost;
    double info_gain;
};

class ERRTExplorer : public rclcpp::Node
{
public:
    ERRTExplorer() : Node("errt_explorer_node")
    {
        RCLCPP_INFO(get_logger(), "ERRT Explorer (Stable Cave Mode)");

        declare_parameter("v_local", 20.0);
        declare_parameter("max_nodes", 1200);
        declare_parameter("step_size", 2.0);
        declare_parameter("k_info", 2.0);
        declare_parameter("k_dist", 0.6);

        get_parameter("v_local", v_local_);
        get_parameter("max_nodes", max_nodes_);
        get_parameter("step_size", step_size_);
        get_parameter("k_info", k_info_);
        get_parameter("k_dist", k_dist_);

        target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10,
            std::bind(&ERRTExplorer::odomCallback, this, std::placeholders::_1));

        map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10,
            std::bind(&ERRTExplorer::mapCallback, this, std::placeholders::_1));

        explore_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10,
            std::bind(&ERRTExplorer::exploreCallback, this, std::placeholders::_1));

        // 🔥 10 Hz control loop
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ERRTExplorer::exploreLoop, this));
    }

private:

    double v_local_, step_size_, k_info_, k_dist_;
    int max_nodes_;

    bool exploration_enabled_ = false;
    bool altitude_locked_ = false;
    double locked_altitude_ = 0.0;

    std::shared_ptr<octomap::OcTree> octree_;
    Vector3d current_pos_;
    double current_yaw_ = 0.0;

    std::vector<ERRTNode> tree_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pos_ = Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree)
            octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
    }

    void exploreCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !exploration_enabled_)
        {
            exploration_enabled_ = true;

            // 🔒 Lock altitude instantly
            locked_altitude_ = current_pos_.z();
            altitude_locked_ = true;

            RCLCPP_INFO(get_logger(),
                        "ERRT taking control. Altitude locked at %.2f",
                        locked_altitude_);

            // 🔥 Immediately publish hover to prevent drop
            publishHover();
        }
    }

    void exploreLoop()
    {
        if (!exploration_enabled_ || !octree_)
            return;

        buildTree();

        auto best = evaluateBest();

        // if (!best.empty())
        //     publishNext(best);
        if (!std::isnan(best.x()))
            publishNext(best);
        else
            publishHover();
    }

    void publishHover()
    {
        publishNext(Vector3d(current_pos_.x(),
                             current_pos_.y(),
                             locked_altitude_));
    }

    void buildTree()
    {
        tree_.clear();
        tree_.push_back({Vector3d(current_pos_.x(),
                                  current_pos_.y(),
                                  locked_altitude_), -1, 0.0, 0.0});

        std::default_random_engine gen(std::random_device{}());
        std::uniform_real_distribution<> dist(-v_local_/2, v_local_/2);

        for (int i = 0; i < max_nodes_; ++i)
        {
            Vector3d sample(
                current_pos_.x() + dist(gen),
                current_pos_.y() + dist(gen),
                locked_altitude_);   // 🔥 CONSTANT ALTITUDE

            int nearest = nearestNode(sample);

            Vector3d dir = (sample - tree_[nearest].position).normalized();
            Vector3d new_pos = tree_[nearest].position + step_size_ * dir;

            new_pos.z() = locked_altitude_;  // 🔥 FORCE ALTITUDE

            if (isFree(new_pos))
            {
                tree_.push_back({new_pos, nearest, 0.0, computeInfoGain(new_pos)});
            }
        }
    }

    int nearestNode(const Vector3d& sample)
    {
        int best = 0;
        double min_d = 1e9;
        // for (int i = 0; i < tree_.size(); ++i)
        for (size_t i = 0; i < tree_.size(); ++i)
        {
            double d = (tree_[i].position - sample).norm();
            if (d < min_d)
            {
                min_d = d;
                best = i;
            }
        }
        return best;
    }

    bool isFree(const Vector3d& pos)
    {
        auto node = octree_->search(pos.x(), pos.y(), pos.z());
        return (!node || !octree_->isNodeOccupied(node));
    }

    double computeInfoGain(const Vector3d& pos)
    {
        return 1.0;  // simplified for stability
    }

    Vector3d evaluateBest()
    {
        if (tree_.size() < 2)
            return Vector3d(current_pos_.x(),
                            current_pos_.y(),
                            locked_altitude_);

        return tree_.back().position;
    }

    void publishNext(const Vector3d& next)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "world";

        msg.pose.position.x = next.x();
        msg.pose.position.y = next.y();
        msg.pose.position.z = next.z();

        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        target_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ERRTExplorer>());
    rclcpp::shutdown();
    return 0;
}