cmake_minimum_required(VERSION 3.5)
project(exploration_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include headers
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ----------------------------
# Lazy Explorer
# ----------------------------
add_executable(lazy_explorer src/lazy_explorer.cc)

ament_target_dependencies(lazy_explorer
  rclcpp
  geometry_msgs
  octomap
  octomap_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
)

# ----------------------------
# Hybrid Frontier Explorer
# ----------------------------
add_executable(hybrid_frontier_explorer src/hybrid_frontier_explorer.cc)

ament_target_dependencies(hybrid_frontier_explorer
  rclcpp
  geometry_msgs
  octomap
  octomap_msgs
  nav_msgs
)

# ----------------------------
# Raycast Explorer
# ----------------------------
add_executable(raycast_explorer src/raycast_explorer.cc)

ament_target_dependencies(raycast_explorer
  rclcpp
  geometry_msgs
  octomap
  octomap_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# ----------------------------
# Raycast Pointcloud Explorer
# ----------------------------
add_executable(raycast_pointcloud_explorer src/raycast_explorer_pointcloud.cc)

ament_target_dependencies(raycast_pointcloud_explorer
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
)

# Link Eigen explicitly (safe practice)
target_link_libraries(hybrid_frontier_explorer Eigen3::Eigen)
target_link_libraries(raycast_explorer Eigen3::Eigen)
target_link_libraries(raycast_pointcloud_explorer Eigen3::Eigen)
target_link_libraries(raycast_pointcloud_explorer ${OpenCV_LIBRARIES})

# ----------------------------
# Install
# ----------------------------
install(TARGETS
  lazy_explorer
  hybrid_frontier_explorer
  raycast_explorer
  raycast_pointcloud_explorer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()