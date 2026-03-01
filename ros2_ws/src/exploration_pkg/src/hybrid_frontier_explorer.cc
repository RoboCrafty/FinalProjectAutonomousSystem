#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <octomap/OcTree.h>
#include <Eigen/Dense>

using std::placeholders::_1;

class HybridFrontierExplorer : public rclcpp::Node
{
public:
  HybridFrontierExplorer()
  : Node("hybrid_frontier_explorer")
  {
    // Parameters
    lookahead_dist_ = this->declare_parameter("lookahead_distance", 5.0);
    safety_dist_ = this->declare_parameter("safety_distance", 1.0);
    fov_deg_ = this->declare_parameter("forward_fov_deg", 180.0);
    wall_gain_ = this->declare_parameter("wall_repulsion_gain", 1.5);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full", 1,
      std::bind(&HybridFrontierExplorer::octomapCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/current_state_est", 10,
      std::bind(&HybridFrontierExplorer::odomCallback, this, _1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&HybridFrontierExplorer::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Hybrid Frontier Explorer started");
  }

private:
  // =============================
  // Callbacks
  // =============================

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);

    double yaw = getYaw(msg->pose.pose.orientation);
    current_yaw_ = yaw;
    odom_received_ = true;
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    octree_.reset(dynamic_cast<octomap::OcTree*>(
      octomap_msgs::fullMsgToMap(*msg)));

    map_received_ = true;
  }

  // =============================
  // Main timer
  // =============================

  void timerCallback()
  {
    if (!odom_received_ || !map_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting for odom/map...");
      return;
    }

    std::vector<Eigen::Vector3d> frontiers = findFrontiers();
    if (frontiers.empty()) {
      RCLCPP_WARN(get_logger(), "No frontiers found");
      return;
    }

    Eigen::Vector3d dir = computeGapDirection(frontiers);

    // Apply soft wall avoidance
    Eigen::Vector3d repel = computeWallRepulsion();
    Eigen::Vector3d final_dir = (dir + repel).normalized();

    publishGoal(final_dir);
  }

  // =============================
  // Frontier detection
  // =============================

  std::vector<Eigen::Vector3d> findFrontiers()
  {
    std::vector<Eigen::Vector3d> frontiers;
    double fov_rad = fov_deg_ * M_PI / 180.0;

    Eigen::Vector2d forward(cos(current_yaw_), sin(current_yaw_));

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it)
    {
      if (octree_->isNodeOccupied(*it))
        continue;

      // check unknown neighbor
      if (!hasUnknownNeighbor(it.getKey()))
        continue;

      Eigen::Vector3d pos(it.getX(), it.getY(), it.getZ());
      Eigen::Vector2d diff = (pos - current_pos_).head<2>();

      if (diff.norm() < 0.5)
        continue;

      diff.normalize();
      double angle = acos(forward.dot(diff));

      if (angle < fov_rad * 0.5)
        frontiers.push_back(pos);
    }

    return frontiers;
  }

  bool hasUnknownNeighbor(const octomap::OcTreeKey & key)
  {
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz)
        {
          octomap::OcTreeKey nkey(key.k[0]+dx, key.k[1]+dy, key.k[2]+dz);
          if (octree_->search(nkey) == nullptr)
            return true;
        }
    return false;
  }

  // =============================
  // Gap direction (distance weighted)
  // =============================

  Eigen::Vector3d computeGapDirection(
      const std::vector<Eigen::Vector3d>& frontiers)
  {
    Eigen::Vector2d accum(0, 0);

    for (const auto & f : frontiers)
    {
      Eigen::Vector2d diff = (f - current_pos_).head<2>();
      double dist = diff.norm();
      if (dist < 0.001) continue;

      diff.normalize();

      // weight by distance (farther = more attractive)
      double weight = std::min(dist, 10.0);
      accum += weight * diff;
    }

    if (accum.norm() < 1e-3)
      return Eigen::Vector3d(cos(current_yaw_), sin(current_yaw_), 0);

    accum.normalize();
    return Eigen::Vector3d(accum.x(), accum.y(), 0);
  }

  // =============================
  // Soft wall repulsion
  // =============================

  Eigen::Vector3d computeWallRepulsion()
  {
    Eigen::Vector2d repel(0, 0);

    double radius = safety_dist_;

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it)
    {
      if (!octree_->isNodeOccupied(*it))
        continue;

      Eigen::Vector2d obs(it.getX(), it.getY());
      Eigen::Vector2d diff = current_pos_.head<2>() - obs;

      double dist = diff.norm();
      if (dist > radius || dist < 0.01)
        continue;

      diff.normalize();
      double strength = wall_gain_ * (radius - dist) / radius;
      repel += strength * diff;
    }

    if (repel.norm() > 1e-3)
      repel.normalize();

    return Eigen::Vector3d(repel.x(), repel.y(), 0);
  }

  // =============================
  // Publish goal
  // =============================

  void publishGoal(const Eigen::Vector3d & dir)
  {
    Eigen::Vector3d goal = current_pos_ + lookahead_dist_ * dir;

    double yaw = atan2(dir.y(), dir.x());

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();

    msg.pose.position.x = goal.x();
    msg.pose.position.y = goal.y();
    msg.pose.position.z = current_pos_.z();

    msg.pose.orientation = yawToQuat(yaw);

    goal_pub_->publish(msg);
  }

  // =============================
  // Helpers
  // =============================

  double getYaw(const geometry_msgs::msg::Quaternion & q)
  {
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
  }

  geometry_msgs::msg::Quaternion yawToQuat(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = cos(yaw * 0.5);
    q.z = sin(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    return q;
  }

  // =============================
  // Members
  // =============================

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<octomap::OcTree> octree_;

  Eigen::Vector3d current_pos_{0,0,0};
  double current_yaw_{0};

  bool map_received_{false};
  bool odom_received_{false};

  double lookahead_dist_;
  double safety_dist_;
  double fov_deg_;
  double wall_gain_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridFrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}