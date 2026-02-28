#include "basic_waypoint_pkg/planner.hpp"

#include <utility>

BasicPlanner::BasicPlanner(const rclcpp::Node::SharedPtr & node)
: node_(node),
  current_pose_(Eigen::Affine3d::Identity()),
  current_velocity_(Eigen::Vector3d::Zero()),
  current_angular_velocity_(Eigen::Vector3d::Zero()),
  max_v_(2.0),
  max_a_(0.5),
  max_ang_v_(0.0),
  max_ang_a_(0.0),
  goal_received_(false),
  odom_received_(false)
{
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  To Do: Load Trajectory Parameters from parameter file (ROS2)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  //
  // ~~~~ begin solution

  //
  //     **** FILL IN HERE (ROS2 parameter handling) ***
  //
  
  node_->declare_parameter("max_v", 5.0);
  node_->declare_parameter("max_a", 2.0);
  // node_->declare_parameter("waypoints_pos", std::vector<std::vector<double>>{});
  // node_->declare_parameter("waypoints_vel", std::vector<std::vector<double>>{});
  // node_->declare_parameter("waypoints_acc", std::vector<std::vector<double>>{});

  if (!node_->get_parameter("max_v", max_v_)) {
    RCLCPP_WARN(node_->get_logger(), "param max_v not found, using default");
  }
  if (!node_->get_parameter("max_a", max_a_)) {
    RCLCPP_WARN(node_->get_logger(), "param max_a not found, using default");
  }
  // ~~~~ end solution

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Publishers
  pub_markers_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_markers", 10);

  pub_trajectory_ =
    node_->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
      "trajectory", 10);

  // Subscriber for Odometry
  sub_odom_ =
    node_->create_subscription<nav_msgs::msg::Odometry>(
      "/current_state_est", 10,
      std::bind(&BasicPlanner::uavOdomCallback, this, std::placeholders::_1));

  sub_goal_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    10,
    std::bind(&BasicPlanner::goalCallback, this, std::placeholders::_1)
);
}

// Goal callback
void BasicPlanner::goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_position_ = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

  goal_orientation_ = Eigen::Quaterniond(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);

  goal_received_ = true;

  RCLCPP_INFO(node_->get_logger(), "Received new goal");

  if (!odom_received_) {
    RCLCPP_WARN(node_->get_logger(), "No odom yet, cannot plan");
    return;
  }

  mav_trajectory_generation::Trajectory trajectory;

  if (planTrajectory(goal_position_, Eigen::Vector3d::Zero(), &trajectory)) {
    publishTrajectory(trajectory);
  }
}

// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // store current position in our planner
  tf2::fromMsg(odom->pose.pose, current_pose_);

  // store current velocity
  tf2::fromMsg(odom->twist.twist.linear, current_velocity_);

  odom_received_ = true;
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v)
{
  max_v_ = max_v;
}

// Plans a trajectory from the current position to a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(
  const Eigen::VectorXd & goal_pos,
  const Eigen::VectorXd & goal_vel,
  mav_trajectory_generation::Trajectory * trajectory)
{
  const int dimension = 4;
  mav_trajectory_generation::Vertex::Vector vertices;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;

  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  double current_yaw = atan2(current_pose_.rotation()(1,0), current_pose_.rotation()(0,0));
  Eigen::Vector4d start_state;
  start_state << current_pose_.translation(), current_yaw;

  /******* Configure start point *******/
  start.makeStartOrEnd(start_state, derivative_to_optimize);
  Eigen::Vector4d start_vel_4d;
  start_vel_4d << current_velocity_, 0.0; // assuming zero angular velocity
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_vel_4d);
  vertices.push_back(start);

  /******* Configure end point *******/
  double goal_yaw = atan2(2.0 * (goal_orientation_.w() * goal_orientation_.z() + goal_orientation_.x() * goal_orientation_.y()),
                          1.0 - 2.0 * (goal_orientation_.y() * goal_orientation_.y() + goal_orientation_.z() * goal_orientation_.z()));

  Eigen::Vector4d end_state;
  end_state << goal_pos, goal_yaw;
  end.makeStartOrEnd(end_state, derivative_to_optimize);

  Eigen::Vector4d goal_vel_4d;
  goal_vel_4d << goal_vel, 0.0; // assuming zero angular velocity at goal
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_vel_4d);
  vertices.push_back(end);

  // estimate segment times
  std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  opt.optimize();
  opt.getTrajectory(trajectory);

  return true;
}

// Overload using explicit start state and limits (currently just a stub, same as above)
bool BasicPlanner::planTrajectory(
  const Eigen::VectorXd & goal_pos,
  const Eigen::VectorXd & goal_vel,
  const Eigen::VectorXd & start_pos,
  const Eigen::VectorXd & start_vel,
  double v_max, double a_max,
  mav_trajectory_generation::Trajectory * trajectory)
{
  // You can either implement a different variant or simply reuse the other method.
  (void)start_pos;
  (void)start_vel;
  (void)v_max;
  (void)a_max;
  return planTrajectory(goal_pos, goal_vel, trajectory);
}

bool BasicPlanner::publishTrajectory(
  const mav_trajectory_generation::Trajectory & trajectory)
{
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::msg::MarkerArray markers;
  double distance = 0.2;  // distance between markers; 0.0 to disable
  std::string frame_id = "world";

  drawMavTrajectory(
    trajectory, distance, frame_id, &markers);
  pub_markers_->publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::msg::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
    trajectory, &msg);
  msg.header.frame_id = "world";
  // optionally: msg.header.stamp = node_->now();
  pub_trajectory_->publish(msg);

  return true;
}

void BasicPlanner::drawMavTrajectory(
    const mav_trajectory_generation::Trajectory& trajectory,
    double distance, const std::string& frame_id,
    visualization_msgs::msg::MarkerArray* marker_array) {
    // sample trajectory
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, 0.1, &trajectory_points);
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Could not sample trajectory.");
        return;
    }

    // draw trajectory
    marker_array->markers.clear();

    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // orange
    std_msgs::msg::ColorRGBA line_strip_color;
    line_strip_color.r = 1.0;
    line_strip_color.g = 0.5;
    line_strip_color.b = 0.0;
    line_strip_color.a = 1.0;
    line_strip.color = line_strip_color;
    line_strip.scale.x = 0.01;
    line_strip.ns = "path";

    double accumulated_distance = 0.0;
    Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
    double scale = 0.3;
    double diameter = 0.3;
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        const mav_msgs::EigenTrajectoryPoint& point = trajectory_points[i];

        accumulated_distance += (last_position - point.position_W).norm();
        if (accumulated_distance > distance) {
            accumulated_distance = 0.0;
            mav_msgs::EigenMavState mav_state;
            mav_msgs::EigenMavStateFromEigenTrajectoryPoint(point, &mav_state);
            mav_state.orientation_W_B = point.orientation_W_B;

            visualization_msgs::msg::MarkerArray axes_arrows;
            axes_arrows.markers.resize(3);

            // x axis
            visualization_msgs::msg::Marker arrow_marker = axes_arrows.markers[0];
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
            arrow_marker.color = color;  // x - red
            arrow_marker.points.resize(2);
            arrow_marker.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitX() * scale)
            );
            arrow_marker.scale.x = diameter * 0.1;
            arrow_marker.scale.y = diameter * 0.2;
            arrow_marker.scale.z = 0;

            // y axis
            visualization_msgs::msg::Marker arrow_marker_y = axes_arrows.markers[1];
            arrow_marker_y.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_y.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color_y;
            color_y.r = 0.0; color_y.g = 1.0; color_y.b = 0.0; color_y.a = 1.0;
            arrow_marker_y.color = color_y;  // y - green
            arrow_marker_y.points.resize(2);
            arrow_marker_y.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker_y.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitY() * scale)
            );
            arrow_marker_y.scale.x = diameter * 0.1;
            arrow_marker_y.scale.y = diameter * 0.2;
            arrow_marker_y.scale.z = 0;

            // z axis
            visualization_msgs::msg::Marker arrow_marker_z = axes_arrows.markers[2];
            arrow_marker_z.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_z.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color_z;
            color_z.r = 0.0; color_z.g = 0.0; color_z.b = 1.0; color_z.a = 1.0;
            arrow_marker_z.color = color_z;  // z - blue
            arrow_marker_z.points.resize(2);
            arrow_marker_z.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker_z.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitZ() * scale)
            );
            arrow_marker_z.scale.x = diameter * 0.1;
            arrow_marker_z.scale.y = diameter * 0.2;
            arrow_marker_z.scale.z = 0;

            // append to marker array
            for (size_t j = 0; j < axes_arrows.markers.size(); ++j) {
                axes_arrows.markers[j].header.frame_id = frame_id;
                axes_arrows.markers[j].ns = "pose";
                marker_array->markers.push_back(axes_arrows.markers[j]);
            }
        }
        last_position = point.position_W;
        geometry_msgs::msg::Point last_position_msg;
        last_position_msg = tf2::toMsg(last_position);
        line_strip.points.push_back(last_position_msg);
    }
    marker_array->markers.push_back(line_strip);

    std_msgs::msg::Header header;
    header.frame_id = frame_id;
    header.stamp = node_->now();
    for (size_t i = 0; i < marker_array->markers.size(); ++i) {
        marker_array->markers[i].header = header;
        marker_array->markers[i].id = i;
        marker_array->markers[i].lifetime = rclcpp::Duration::from_seconds(0.0);
        marker_array->markers[i].action = visualization_msgs::msg::Marker::ADD;
    }
}
