#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include <octomap/octomap.h>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <Eigen/Eigenvalues>
#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;

class TubeCenterExplorer : public rclcpp::Node
{
public:
  TubeCenterExplorer()
  : Node("tube_center_explorer"),
    octree_(nullptr)
  {
    step_distance_  = this->declare_parameter<double>("step_distance", 20.0);
    tube_radius_    = this->declare_parameter<double>("tube_radius", 3.0);
    tube_length_    = this->declare_parameter<double>("tube_length", 100.0);
    frame_id_       = this->declare_parameter<std::string>("frame_id", "world");

    ray_target_distance_ = this->declare_parameter<double>("ray_target_distance", 20.0);
    ray_tolerance_       = this->declare_parameter<double>("ray_tolerance", 10.0);
    ray_yaw_span_deg_    = this->declare_parameter<double>("ray_yaw_span_deg", 120.0);
    ray_pitch_span_deg_  = this->declare_parameter<double>("ray_pitch_span_deg", 50.0);
    ray_samples_yaw_     = this->declare_parameter<int>("ray_samples_yaw", 25);
    ray_samples_pitch_   = this->declare_parameter<int>("ray_samples_pitch", 15);
    std::string goal_topic  = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    std::string state_topic = this->declare_parameter<std::string>("state_topic", "/current_state_est");
    std::string octomap_topic = this->declare_parameter<std::string>("octomap_topic", "/octomap_full");

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

    trigger_distance_ = this->declare_parameter<double>("trigger_distance", 5.0);
    first_goal_sent_ = false;
    auto octomap_qos = rclcpp::QoS(1).transient_local().reliable();
    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic, 10,
      std::bind(&TubeCenterExplorer::stateCallback, this, std::placeholders::_1));

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, octomap_qos,
      std::bind(&TubeCenterExplorer::octomapCallback, this, std::placeholders::_1));
    
    goal_timer_ = this->create_wall_timer(
    3s,   // every second
    std::bind(&TubeCenterExplorer::computeAndPublishNextGoal, this));

    param_cb_ = this->add_on_set_parameters_callback(
  std::bind(&TubeCenterExplorer::onParamChange, this, std::placeholders::_1));


    RCLCPP_INFO(get_logger(), "TubeCenterExplorer started");
  }

private:

    rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params)
    {
    for (const auto & p : params) {
        if (p.get_name() == "step_distance") {
        step_distance_ = p.as_double();
        }
        else if (p.get_name() == "ray_target_distance") {
        ray_target_distance_ = p.as_double();
        }
        else if (p.get_name() == "ray_tolerance") {
        ray_tolerance_ = p.as_double();
        }
        else if (p.get_name() == "ray_yaw_span_deg") {
        ray_yaw_span_deg_ = p.as_double();
        }
        else if (p.get_name() == "ray_pitch_span_deg") {
        ray_pitch_span_deg_ = p.as_double();
        }
        else if (p.get_name() == "ray_samples_yaw") {
        ray_samples_yaw_ = p.as_int();
        }
        else if (p.get_name() == "ray_samples_pitch") {
        ray_samples_pitch_ = p.as_int();
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
    }
    Eigen::Matrix3d quatToRot(const geometry_msgs::msg::Quaternion & q)
    {
    Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
    eq.normalize();   // VERY important for safety
    return eq.toRotationMatrix();
    }
    void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;
    latest_pose_ = pose_stamped;

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000, 
                        "Pose ready. first_goal_sent_=%d | octree=%p", 
                        first_goal_sent_, octree_);

    // ONLY compute if we have BOTH pose AND OctoMap!
    if (!latest_pose_.has_value() || !octree_) {
        return;  // Wait patiently...
    }

    if (current_goal_.has_value()) {
        double dist = distance3D(latest_pose_->pose.position, current_goal_->pose.position);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                            "📏 Distance to goal: %.2f / %.2f m", dist, trigger_distance_);
        if (dist <= trigger_distance_) {
        RCLCPP_INFO(get_logger(), "🎯 REACHED GOAL ZONE - computing next!");
        computeAndPublishNextGoal();
        }
    } 
    else if (!first_goal_sent_) {
        RCLCPP_INFO(get_logger(), "🚀 FIRST GOAL TRIGGERED!");
        computeAndPublishNextGoal();
        first_goal_sent_ = true;
    }
    }


  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Got OctoMap");
    // Convert message to octomap::OcTree
    if (octree_) {
      delete octree_;
      octree_ = nullptr;
    }

    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
    if (!tree) {
      RCLCPP_WARN(get_logger(), "Failed to deserialize OctoMap");
      return;
    }

    octree_ = dynamic_cast<octomap::OcTree*>(tree);
    if (!octree_) {
      RCLCPP_WARN(get_logger(), "Received OctoMap is not an OcTree");
      delete tree;
    }
  }

  double distance3D(const geometry_msgs::msg::Point & a,
                    const geometry_msgs::msg::Point & b)
  {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }



  

  // --- Helper functions ---

  double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  geometry_msgs::msg::Quaternion yawToQuat(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sy;
    q.w = cy;
    return q;
  }

  void sampleFreeVoxelsInTube(const geometry_msgs::msg::Point & pos,
                              const Eigen::Vector3d & forward,
                              std::vector<Eigen::Vector3d> & out)
  {
    out.clear();
    if (!octree_) return;

    Eigen::Vector3d p0(pos.x, pos.y, pos.z);

    for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        Eigen::Vector3d v(it.getX(), it.getY(), it.getZ());

        Eigen::Vector3d rel = v - p0;
        double s = rel.dot(forward);
        if (s < 0.0 || s > tube_length_) continue;  // not in front or beyond tube length

        // Distance to centerline
        Eigen::Vector3d proj = p0 + s * forward;
        double radial = (v - proj).norm();
        if (radial > tube_radius_) continue;

        out.push_back(v);
      }
    }
  }

  Eigen::Vector3d estimatePrincipalDirection(const std::vector<Eigen::Vector3d> & points,
                                             const Eigen::Vector3d & mean)
  {
    if (points.size() < 3) {
      return Eigen::Vector3d(0, 0, 0);
    }

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (auto & p : points) {
      Eigen::Vector3d d = p - mean;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(points.size());

    // Eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d evals = es.eigenvalues();
    Eigen::Matrix3d evecs = es.eigenvectors();

    // Principal component = eigenvector with largest eigenvalue
    int idx;
    evals.maxCoeff(&idx);
    Eigen::Vector3d dir = evecs.col(idx);
    return dir;
  }

  void computeAndPublishNextGoal()
{
  RCLCPP_INFO(get_logger(), "🔄 COMPUTING NEW GOAL (RAYCAST MODE)");

  if (!latest_pose_.has_value() || !octree_) {
    RCLCPP_WARN(get_logger(), "Cannot compute goal: missing pose or map");
    return;
  }

  auto current = latest_pose_.value();

  Eigen::Vector3d origin(
    current.pose.position.x,
    current.pose.position.y,
    current.pose.position.z);

  // ================================
  // Get camera forward direction
  // ================================
  Eigen::Matrix3d R = quatToRot(current.pose.orientation);
  Eigen::Vector3d forward = R * Eigen::Vector3d::UnitX();

  RCLCPP_INFO(get_logger(),
    "📍 Drone position: (%.2f, %.2f, %.2f)",
    origin.x(), origin.y(), origin.z());

  // ================================
  // Ray casting
  // ================================
  std::vector<Eigen::Vector3d> hits;

  double yaw_span = ray_yaw_span_deg_ * M_PI / 180.0;
  double pitch_span = ray_pitch_span_deg_ * M_PI / 180.0;

  int rays_cast = 0;
  int rays_hit = 0;

  for (int i = 0; i < ray_samples_yaw_; ++i) {
    double yaw_offset =
      -yaw_span/2.0 + yaw_span * (double(i) / (ray_samples_yaw_ - 1));

    for (int j = 0; j < ray_samples_pitch_; ++j) {
      double pitch_offset =
        -pitch_span/2.0 + pitch_span * (double(j) / (ray_samples_pitch_ - 1));

      // direction in body frame
      Eigen::Vector3d dir_body(
        std::cos(pitch_offset) * std::cos(yaw_offset),
        std::cos(pitch_offset) * std::sin(yaw_offset),
        std::sin(pitch_offset));

      // rotate to world
      Eigen::Vector3d dir_world = R * dir_body;
      dir_world.normalize();

      rays_cast++;

      // Raycast
      octomap::point3d origin_o(origin.x(), origin.y(), origin.z());
      octomap::point3d direction_o(dir_world.x(), dir_world.y(), dir_world.z());
      octomap::point3d end;

      bool hit = octree_->castRay(
        origin_o,
        direction_o,
        end,
        true,
        ray_target_distance_ + ray_tolerance_);

      if (!hit) continue;

      rays_hit++;

      Eigen::Vector3d hit_pt(end.x(), end.y(), end.z());
      double dist = (hit_pt - origin).norm();

      // keep only hits near desired ring
      if (std::abs(dist - ray_target_distance_) <= ray_tolerance_) {
        hits.push_back(hit_pt);
      }
    }
  }

  RCLCPP_INFO(get_logger(),
    "📡 Rays cast: %d | hits: %d | ring points: %zu",
    rays_cast, rays_hit, hits.size());

  if (hits.size() < 5) {
    RCLCPP_WARN(get_logger(), "❌ Not enough ring points!");
    return;
  }

  // ================================
  // Compute centroid
  // ================================
  Eigen::Vector3d centroid(0,0,0);
  for (auto & p : hits) centroid += p;
  centroid /= double(hits.size());

  RCLCPP_INFO(get_logger(),
    "🎯 Ring centroid: (%.2f, %.2f, %.2f)",
    centroid.x(), centroid.y(), centroid.z());

  // ================================
  // Direction to centroid
  // ================================
  Eigen::Vector3d to_centroid = centroid - origin;
  double distance = to_centroid.norm();

  if (distance < 1e-3) {
    RCLCPP_WARN(get_logger(), "Centroid too close");
    return;
  }

  to_centroid.normalize();

  double goal_yaw = std::atan2(to_centroid.y(), to_centroid.x());

  // ================================
  // Step forward toward centroid
  // ================================
  Eigen::Vector3d goal_pos =
    origin + to_centroid * step_distance_;

  // ================================
  // Publish goal
  // ================================
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = this->get_clock()->now();
  goal.header.frame_id = frame_id_;

  goal.pose.position.x = goal_pos.x();
  goal.pose.position.y = goal_pos.y();
  goal.pose.position.z = goal_pos.z();
  goal.pose.orientation = yawToQuat(goal_yaw);

  RCLCPP_INFO(get_logger(),
    "✅ GOAL → pos(%.2f %.2f %.2f) yaw=%.2f deg",
    goal.pose.position.x,
    goal.pose.position.y,
    goal.pose.position.z,
    goal_yaw * 180.0 / M_PI);

  goal_pub_->publish(goal);
  current_goal_ = goal;
}

  // Members
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> current_goal_;
  double trigger_distance_;  // e.g. 5.0
  bool first_goal_sent_;

  octomap::OcTree * octree_;

  double step_distance_;
  double tube_radius_;
  double tube_length_;
  std::string frame_id_;

  double ray_target_distance_;
  double ray_tolerance_;
  double ray_yaw_span_deg_;
  double ray_pitch_span_deg_;
  int ray_samples_yaw_;
  int ray_samples_pitch_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TubeCenterExplorer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
