#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <random>

using namespace std::chrono_literals;

struct RRTNode {
    octomap::point3d pos;
    int parent;
    double gain;
};

class ERRTExplorer3D : public rclcpp::Node {
public:
    ERRTExplorer3D() : Node("nbv_explorer_3d_node"), is_exploring_(false) {
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_exploration", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { is_exploring_ = msg->data; });
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_ = octomap::point3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
                has_odom_ = true;
            });

        sub_map_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
                octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                if (tree) octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
            });

        pub_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        timer_ = this->create_wall_timer(800ms, std::bind(&ERRTExplorer3D::explorationLoop, this));
    }

private:
    void explorationLoop() {
        if (!is_exploring_ || !octree_ || !has_odom_) return;

        std::vector<RRTNode> tree;
        tree.push_back({current_pos_, -1, 0.0});

        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<> x_d(current_pos_.x()-15, current_pos_.x()+15);
        std::uniform_real_distribution<> y_d(current_pos_.y()-15, current_pos_.y()+15);
        std::uniform_real_distribution<> z_d(current_pos_.z()-5, current_pos_.z()+5);

        for(int i=0; i<80; ++i) {
            octomap::point3d q_rand(x_d(gen), y_d(gen), z_d(gen));
            int near = 0; double min_d = 1e6;
            for(size_t j=0; j<tree.size(); ++j) {
                double d = (tree[j].pos - q_rand).norm();
                if(d < min_d) { min_d = d; near = j; }
            }

            double step = 3.5;
            octomap::point3d dir = (q_rand - tree[near].pos).normalized();
            octomap::point3d q_new = tree[near].pos + dir * step;

            octomap::point3d hit;
            if(!octree_->castRay(tree[near].pos, dir, hit, true, step + 0.5)) {
                double g = calculateGain(q_new);
                tree.push_back({q_new, near, tree[near].gain + g});
            }
        }

        int best_idx = -1; double max_g = 5.0;
        for(size_t i=1; i<tree.size(); ++i) {
            if(tree[i].gain > max_g) { max_g = tree[i].gain; best_idx = i; }
        }

        if(best_idx != -1) {
            // Trace back to find the FIRST step from the drone
            int curr = best_idx;
            while(tree[curr].parent != 0 && tree[curr].parent != -1) {
                curr = tree[curr].parent;
            }
            publishSetpoint(tree[curr].pos);
        }
    }

    double calculateGain(octomap::point3d p) {
        double g = 0; double r = 4.0;
        for(double x=-r; x<=r; x+=1.2)
            for(double y=-r; y<=r; y+=1.2)
                if(!octree_->search(p.x()+x, p.y()+y, p.z())) g += 1.0;
        return g;
    }

    void publishSetpoint(octomap::point3d p) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";
        msg.pose.position.x = p.x();
        msg.pose.position.y = p.y();
        msg.pose.position.z = p.z();
        
        double yaw = std::atan2(p.y() - current_pos_.y(), p.x() - current_pos_.x());
        tf2::Quaternion q; q.setRPY(0, 0, yaw);
        msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
        
        pub_target_->publish(msg);
    }

    std::shared_ptr<octomap::OcTree> octree_;
    octomap::point3d current_pos_;
    bool is_exploring_, has_odom_ = false;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ERRTExplorer3D>());
    rclcpp::shutdown();
    return 0;
}