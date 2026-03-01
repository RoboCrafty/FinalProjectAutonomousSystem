#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode() : Node("trajectory_generator_node") {
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
                if (!msg->poses.empty()) {
                    // Send the second point in the path (the next step)
                    auto target = (msg->poses.size() > 1) ? msg->poses[1] : msg->poses[0];
                    pub_setpoint_->publish(target);
                }
            });
        pub_setpoint_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_setpoint_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}