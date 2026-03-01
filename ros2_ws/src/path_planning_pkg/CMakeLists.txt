cmake_minimum_required(VERSION 3.8)
project(path_planning_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# Find Dependencies
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# The actual Octomap C++ library (required to read the map)
find_package(octomap REQUIRED)

# University custom packages for trajectory generation
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_planning_msgs REQUIRED)

# --> NEW: Find Eigen3 <--
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------
# Executables and Linking
# ---------------------------------------------------------
add_executable(path_planner_node src/path_planner_node.cpp)

# Link the ROS2 dependencies
ament_target_dependencies(path_planner_node 
  rclcpp 
  octomap_msgs 
  geometry_msgs 
  nav_msgs 
  tf2_ros
  mav_trajectory_generation
  mav_planning_msgs
)

# Link the standard Octomap C++ dependencies AND Eigen3
target_include_directories(path_planner_node PUBLIC 
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} # --> NEW: Add Eigen to include directories <--
)
target_link_libraries(path_planner_node ${OCTOMAP_LIBRARIES})

# ---------------------------------------------------------
# Install Targets
# ---------------------------------------------------------
install(TARGETS path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
