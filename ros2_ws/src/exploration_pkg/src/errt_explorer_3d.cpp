#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <tf2/LinearMath/Quaternion.h>
#include <random>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

struct TreeNode {
    double x, y, z;
    int parent;
};

class ERRTExplorer3D : public rclcpp::Node {
public:
    ERRTExplorer3D() : Node("errt_explorer_3d_node"),
                       is_exploring_(false),
                       has_odom_(false),
                       step_size_(3.0),
                       max_iterations_(400)
    {
        sub_enable_ = create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10,
            std::bind(&ERRTExplorer3D::enableCb, this, std::placeholders::_1));

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10,
            std::bind(&ERRTExplorer3D::odomCb, this, std::placeholders::_1));

        sub_map_ = create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10,
            std::bind(&ERRTExplorer3D::mapCb, this, std::placeholders::_1));

        pub_target_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/next_setpoint", 10);

        timer_ = create_wall_timer(800ms,
            std::bind(&ERRTExplorer3D::explorationLoop, this));

        RCLCPP_INFO(get_logger(), "ERRT Explorer Ready.");
    }

private:
    void enableCb(const std_msgs::msg::Bool::SharedPtr msg) {
        is_exploring_ = msg->data;
    }

    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    void mapCb(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
        }
    }

    bool collisionFree(double x1, double y1, double z1,
                       double x2, double y2, double z2) {
        octomap::point3d origin(x1, y1, z1);
        octomap::point3d direction(x2 - x1, y2 - y1, z2 - z1);
        octomap::point3d hit;
        return !octree_->castRay(origin, direction, hit, true, step_size_);
    }

    void explorationLoop() {
        if (!is_exploring_ || !octree_ || !has_odom_) return;

        tree_.clear();
        tree_.push_back({current_x_, current_y_, current_z_, -1});

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dx(current_x_ - 20, current_x_ + 20);
        std::uniform_real_distribution<> dy(current_y_ - 20, current_y_ + 20);
        std::uniform_real_distribution<> dz(current_z_ - 10, current_z_ + 10);

        double best_x = current_x_;
        double best_y = current_y_;
        double best_z = current_z_;
        bool found_frontier = false;

        for (int i = 0; i < max_iterations_; ++i) {

            double rx = dx(gen);
            double ry = dy(gen);
            double rz = dz(gen);

            int nearest = 0;
            double min_dist = 1e9;

            for (int j = 0; j < tree_.size(); ++j) {
                double dist = pow(tree_[j].x - rx, 2) +
                              pow(tree_[j].y - ry, 2) +
                              pow(tree_[j].z - rz, 2);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest = j;
                }
            }

            double vx = rx - tree_[nearest].x;
            double vy = ry - tree_[nearest].y;
            double vz = rz - tree_[nearest].z;
            double norm = sqrt(vx*vx + vy*vy + vz*vz);
            if (norm == 0) continue;

            vx = (vx / norm) * step_size_;
            vy = (vy / norm) * step_size_;
            vz = (vz / norm) * step_size_;

            double new_x = tree_[nearest].x + vx;
            double new_y = tree_[nearest].y + vy;
            double new_z = tree_[nearest].z + vz;

            if (collisionFree(tree_[nearest].x, tree_[nearest].y,
                              tree_[nearest].z,
                              new_x, new_y, new_z)) {

                tree_.push_back({new_x, new_y, new_z, nearest});

                if (octree_->search(new_x, new_y, new_z) == nullptr) {
                    best_x = new_x;
                    best_y = new_y;
                    best_z = new_z;
                    found_frontier = true;
                    break;
                }
            }
        }

        if (!found_frontier) return;

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "world";
        msg.pose.position.x = best_x;
        msg.pose.position.y = best_y;
        msg.pose.position.z = best_z;

        tf2::Quaternion q;
        q.setRPY(0, 0, atan2(best_y - current_y_,
                             best_x - current_x_));
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        pub_target_->publish(msg);

        RCLCPP_INFO(get_logger(),
                    "ERRT target: %.2f %.2f %.2f",
                    best_x, best_y, best_z);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<octomap::OcTree> octree_;
    std::vector<TreeNode> tree_;

    bool is_exploring_, has_odom_;
    double current_x_, current_y_, current_z_;
    double step_size_;
    int max_iterations_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ERRTExplorer3D>());
    rclcpp::shutdown();
    return 0;
}