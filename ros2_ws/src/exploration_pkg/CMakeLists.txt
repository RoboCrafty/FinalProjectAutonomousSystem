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
# 2. Build the Frontier Explorer Node
# ==========================================
# Executable name: frontier_explorer_node
# Source file: src/frontier_explorer_node.cpp
add_executable(frontier_explorer_node src/frontier_explorer_node.cpp)

target_include_directories(frontier_explorer_node PUBLIC 
  ${OCTOMAP_INCLUDE_DIRS}
)

ament_target_dependencies(frontier_explorer_node
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  octomap_msgs
)

target_link_libraries(frontier_explorer_node ${OCTOMAP_LIBRARIES})

# ==========================================
# 3. Build the NBV Explorer Node (Optional)
# ==========================================
add_executable(nbv_explorer_3d_node src/nbv_explorer_3d.cpp)
target_include_directories(nbv_explorer_3d_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(nbv_explorer_3d_node
  rclcpp std_msgs geometry_msgs nav_msgs 
  tf2 tf2_ros tf2_geometry_msgs octomap_msgs
)
target_link_libraries(nbv_explorer_3d_node ${OCTOMAP_LIBRARIES})

# ==========================================
# 4. Install Executables
# ==========================================
install(TARGETS
  frontier_explorer_node
  nbv_explorer_3d_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()