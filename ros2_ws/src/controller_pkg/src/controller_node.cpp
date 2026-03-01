#include "controller_pkg/controller_node.hpp"

#include <chrono>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using namespace std::chrono_literals;

// === Static utility functions =================================================

Eigen::Vector3d ControllerNode::Vee(const Eigen::Matrix3d & in)
{
  Eigen::Vector3d out;
  out << in(2, 1), in(0, 2), in(1, 0);
  return out;
}

double ControllerNode::signed_sqrt(double val)
{
  return val > 0.0 ? std::sqrt(val) : -std::sqrt(-val);
}

// === Constructor ===============================================================

ControllerNode::ControllerNode()
: rclcpp::Node("controller_node"),
  e3(0.0, 0.0, 1.0),
  F2W(4, 4),
  hz(1000.0)
{
  received_desired = false;

  // Subscribers
  desired_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    "command/trajectory", rclcpp::QoS(10),
    std::bind(&ControllerNode::onDesiredState, this, std::placeholders::_1));

  current_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "current_state", rclcpp::QoS(10),
    std::bind(&ControllerNode::onCurrentState, this, std::placeholders::_1));

  // Publisher
  motor_pub_ = this->create_publisher<mav_msgs::msg::Actuators>(
    "rotor_speed_cmds", rclcpp::QoS(10));

  // Timer at hz Hz
  auto period = std::chrono::duration<double>(1.0 / hz);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ControllerNode::controlLoop, this));

  // Controller gains (you can move to parameters if desired)
  this->declare_parameter<double>("kx", 13.1);
  this->declare_parameter<double>("kv", 5.8);
  this->declare_parameter<double>("kr", 9.2);
  this->declare_parameter<double>("komega", 1.2);
  kx = this->get_parameter("kx").as_double();
  kv = this->get_parameter("kv").as_double();
  kr = this->get_parameter("kr").as_double();
  komega = this->get_parameter("komega").as_double();

  RCLCPP_INFO(this->get_logger(),
              "Controller gains: kx=%.3f, kv=%.3f, kr=%.3f, komega=%.3f",
              kx, kv, kr, komega);

  // Physical constants
  m = 1.0;
  cd = 1e-5;
  cf = 1e-3;
  g = 9.81;
  d = 0.3;
  J << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0;

  RCLCPP_INFO(this->get_logger(), "controller_node ready (hz=%.1f)", hz);
}

// === Callbacks ================================================================

void ControllerNode::onDesiredState(
  const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr des_state_msg)
{
  const auto & point = des_state_msg->points[0];

  geometry_msgs::msg::Vector3 pos =
    point.transforms[0].translation;
  xd << pos.x, pos.y, pos.z;

  geometry_msgs::msg::Vector3 vel =
    point.velocities[0].linear;
  vd << vel.x, vel.y, vel.z;

  geometry_msgs::msg::Vector3 acc =
    point.accelerations[0].linear;
  ad << acc.x, acc.y, acc.z;

  geometry_msgs::msg::Quaternion quat =
    point.transforms[0].rotation;
  Eigen::Quaterniond q_des(quat.w, quat.x, quat.y, quat.z);
  Eigen::Matrix3d R_des = q_des.toRotationMatrix();
  yawd = std::atan2(R_des(1, 0), R_des(0, 0));

  received_desired = true;
}

void ControllerNode::onCurrentState(
  const nav_msgs::msg::Odometry::SharedPtr cur_state_msg)
{
  geometry_msgs::msg::Point pos = cur_state_msg->pose.pose.position;
  x << pos.x, pos.y, pos.z;

  geometry_msgs::msg::Vector3 vel = cur_state_msg->twist.twist.linear;
  v << vel.x, vel.y, vel.z;

  geometry_msgs::msg::Quaternion quat = cur_state_msg->pose.pose.orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  R = q.toRotationMatrix();

  geometry_msgs::msg::Vector3 ang_vel = cur_state_msg->twist.twist.angular;
  Eigen::Vector3d omega_world(ang_vel.x, ang_vel.y, ang_vel.z);
  omega = R.transpose() * omega_world;
}

// === Control loop =============================================================

void ControllerNode::controlLoop()
{
  if (!received_desired) {
    return;
  }

  Eigen::Vector3d ex, ev, er, eomega;

  ex = x - xd;
  ev = v - vd;

  Eigen::Vector3d a = -kx * ex - kv * ev + m * g * e3 + m * ad;
  Eigen::Vector3d b3d = a.normalized();

  Eigen::Vector3d yaw_vec(std::cos(yawd), std::sin(yawd), 0.0);
  Eigen::Vector3d b2d = b3d.cross(yaw_vec).normalized();
  Eigen::Vector3d b1d = b2d.cross(b3d).normalized();

  Eigen::Matrix3d Rd;
  Rd << b1d, b2d, b3d;

  Eigen::Matrix3d eR_mat = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
  er = Vee(eR_mat);
  eomega = -omega;

  double F = a.dot(R * e3);
  Eigen::Vector3d M = -kr * er - komega * eomega + omega.cross(J * omega);

  Eigen::Vector4d wrench;
  wrench << F, M(0), M(1), M(2);

  double d_hat = d / std::sqrt(2.0);
  Eigen::Matrix4d F2W_local;
  F2W_local << cf,         cf,         cf,         cf,
               cf * d_hat, cf * d_hat, -cf * d_hat, -cf * d_hat,
              -cf * d_hat, cf * d_hat,  cf * d_hat, -cf * d_hat,
               cd,        -cd,         cd,        -cd;

  Eigen::Vector4d omega_squared = F2W_local.inverse() * wrench;

  mav_msgs::msg::Actuators cmd;
  cmd.angular_velocities.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    cmd.angular_velocities[i] = signed_sqrt(omega_squared(i));
  }

  motor_pub_->publish(cmd);
}
