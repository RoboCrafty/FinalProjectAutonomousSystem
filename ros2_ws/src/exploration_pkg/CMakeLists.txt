cmake_minimum_required(VERSION 3.8)
project(exploration_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

# ==========================================
# 2. Build the Original Frontier Explorer Node
# ==========================================
add_executable(frontier_explorer_node src/frontier_explorer_node.cpp)
target_include_directories(frontier_explorer_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(frontier_explorer_node
  rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs octomap_msgs
)
target_link_libraries(frontier_explorer_node ${OCTOMAP_LIBRARIES})

# ==========================================
# 3. Build the NEW Sappu Frontier Explorer Node
# ==========================================
# This adds your new node while keeping the others
add_executable(sappu_frontier_explorer_node src/sappu_frontier_explorer.cpp)
target_include_directories(sappu_frontier_explorer_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(sappu_frontier_explorer_node
  rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs octomap_msgs
)
target_link_libraries(sappu_frontier_explorer_node ${OCTOMAP_LIBRARIES})

# ==========================================
# 4. Build the NBV Explorer Node
# ==========================================
add_executable(nbv_explorer_3d_node src/nbv_explorer_3d.cpp)
target_include_directories(nbv_explorer_3d_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(nbv_explorer_3d_node
  rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs octomap_msgs
)
target_link_libraries(nbv_explorer_3d_node ${OCTOMAP_LIBRARIES})

# ==========================================
# 5. Install Executables
# ==========================================
# Make sure to add the new node to this list so it is installed correctly
install(TARGETS
  frontier_explorer_node
  sappu_frontier_explorer_node
  nbv_explorer_3d_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
