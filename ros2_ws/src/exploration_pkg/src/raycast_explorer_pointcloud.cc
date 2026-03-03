#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <limits>

class TubeGoalNode : public rclcpp::Node
{
public:
    TubeGoalNode()
    : Node("tube_goal_node"), cam_info_received_(false), odom_received_(false)
    {
        // Parameters
        depth_topic_ = declare_parameter("depth_topic", "/realsense/depth/image");
        camera_info_topic_ = declare_parameter("camera_info_topic", "/realsense/depth/camera_info");
        goal_topic_ = declare_parameter("goal_topic", "/goal_pose");
        odom_topic_ = declare_parameter("odom_topic", "/current_state_est");

        lookahead_distance_ = declare_parameter("lookahead_distance", 5.0);
        max_depth_threshold_ = declare_parameter("max_depth_threshold", 50.0);
        min_valid_pixels_ = declare_parameter("min_valid_pixels", 500);
        min_goal_distance_ = declare_parameter("min_goal_distance", 0.5); // meters

        // Subscribers
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, 10,
            std::bind(&TubeGoalNode::depthCallback, this, std::placeholders::_1));

        cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 10,
            std::bind(&TubeGoalNode::cameraInfoCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&TubeGoalNode::odomCallback, this, std::placeholders::_1));

        // Publisher
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);

        // OpenCV window for debug mask
        cv::namedWindow("mask", cv::WINDOW_NORMAL);
        cv::resizeWindow("mask", 640, 480);

        // Initialize last_goal_ far away so first goal always publishes
        last_goal_ = Eigen::Vector3d(
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest()
        );

        RCLCPP_INFO(get_logger(), "Tube Goal Node Started");
    }

private:
    // Camera info callback
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        cam_info_received_ = true;
    }

    // Odometry callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        odom_received_ = true;
    }

    // Depth callback
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!cam_info_received_ || !odom_received_) return;

        // Convert depth to meters (16UC1 → float)
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_mm = cv_ptr->image;
        if (depth_mm.empty()) return;

        cv::Mat depth;
        depth_mm.convertTo(depth, CV_32F, 0.001);  // mm → meters

        // ---------- STEP 1: mask unknown/far ----------
        cv::Mat valid_mask = (depth > 0.0f) & (depth <= max_depth_threshold_);
        valid_mask.convertTo(valid_mask, CV_8U, 255);  // 0 or 255
        cv::morphologyEx(valid_mask, valid_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, {7,7}));

        // Debug visualization
        cv::imshow("mask", valid_mask);
        cv::waitKey(1);

        // ---------- STEP 2: compute centroid ----------
        cv::Moments m = cv::moments(valid_mask, true);
        if (m.m00 < min_valid_pixels_) return;

        double u = m.m10 / m.m00;
        double v = m.m01 / m.m00;

        // ---------- STEP 3: back-project to camera frame ----------
        double x_cam = (u - cx_) / fx_;
        double y_cam = (v - cy_) / fy_;
        double z_cam = 1.0;
        Eigen::Vector3d dir_cam(x_cam, y_cam, z_cam);
        dir_cam.normalize();

        // ---------- STEP 4: scale by actual depth ----------
        float depth_at_centroid = depth.at<float>(int(v), int(u));
        if (depth_at_centroid <= 0.0f) return; // skip invalid depth
        double scale = std::min((double)depth_at_centroid, lookahead_distance_);

        // ---------- STEP 5: transform to world frame ----------
        Eigen::Quaterniond q_world(
            current_pose_.orientation.w,
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z
        );
        Eigen::Vector3d pos_world(
            current_pose_.position.x,
            current_pose_.position.y,
            current_pose_.position.z
        );

        Eigen::Vector3d camera_offset(0.0, 0.0, 0.0); // adjust if needed
        Eigen::Vector3d goal_pos_world = pos_world + q_world * (dir_cam * scale) + camera_offset;

        // ---------- STEP 6: compute angle ----------
        Eigen::Vector3d drone_forward = q_world * Eigen::Vector3d(1,0,0);  // drone x-axis in world frame
        Eigen::Vector3d goal_dir = (goal_pos_world - pos_world).normalized();
        double angle = acos(drone_forward.dot(goal_dir)) * 180.0 / M_PI;

        // ---------- STEP 7: debug print ----------
        Eigen::Vector3d cam_vec = dir_cam * scale;
        RCLCPP_INFO(get_logger(),
            "Centroid pixel (u,v): (%.1f, %.1f) | Depth: %.2f m | "
            "Camera vec: [%.2f %.2f %.2f] | World vec: [%.2f %.2f %.2f] | "
            "Angle to goal: %.1f deg",
            u, v, depth_at_centroid,
            cam_vec.x(), cam_vec.y(), cam_vec.z(),
            goal_pos_world.x(), goal_pos_world.y(), goal_pos_world.z(),
            angle
        );

        // ---------- STEP 8: publish only if far enough ----------
        if ((goal_pos_world - last_goal_).norm() > min_goal_distance_) {
            geometry_msgs::msg::PoseStamped goal;
            goal.header.stamp = now();
            goal.header.frame_id = "world";
            goal.pose.position.x = goal_pos_world.x();
            goal.pose.position.y = goal_pos_world.y();
            goal.pose.position.z = goal_pos_world.z();
            goal.pose.orientation.w = 1.0;

            goal_pub_->publish(goal);
            last_goal_ = goal_pos_world;
            RCLCPP_INFO(get_logger(), "Published new goal.");
        } else {
            RCLCPP_INFO(get_logger(), "Goal unchanged, not publishing.");
        }
    }

    // -------------------- Members --------------------
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string odom_topic_;
    std::string goal_topic_;

    double lookahead_distance_;
    double max_depth_threshold_;
    int min_valid_pixels_;
    double min_goal_distance_;

    double fx_, fy_, cx_, cy_;
    bool cam_info_received_;
    bool odom_received_;

    geometry_msgs::msg::Pose current_pose_;
    Eigen::Vector3d last_goal_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TubeGoalNode>());
    rclcpp::shutdown();
    return 0;
}