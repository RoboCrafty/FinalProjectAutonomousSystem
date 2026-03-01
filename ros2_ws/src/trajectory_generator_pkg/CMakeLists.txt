cmake_minimum_required(VERSION 3.8)
project(trajectory_generator_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find standard dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Find university custom libraries
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_planning_msgs REQUIRED)

# Create the executable
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)

# Link dependencies (tf2 removed)
ament_target_dependencies(trajectory_generator_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  mav_trajectory_generation
  mav_planning_msgs
)

# Add Eigen includes
target_include_directories(trajectory_generator_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

# Install the node so ROS can run it
install(TARGETS
  trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
