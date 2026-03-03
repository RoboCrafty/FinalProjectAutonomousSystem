cmake_minimum_required(VERSION 3.8)
project(exploration_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED) # The native C++ library
find_package(visualization_msgs REQUIRED)

include_directories(include ${OCTOMAP_INCLUDE_DIRS})

add_executable(frontier_detector_node src/frontier_detector.cpp)
ament_target_dependencies(frontier_detector_node
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_geometry_msgs octomap_msgs
)
target_link_libraries(frontier_detector_node ${OCTOMAP_LIBRARIES})
# ==========================================
# 1. First Brain: NBV Explorer (Breadcrumbs)
# ==========================================
add_executable(nbv_explorer_3d_node src/nbv_explorer_3d.cpp)
target_include_directories(nbv_explorer_3d_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(nbv_explorer_3d_node
  rclcpp std_msgs geometry_msgs nav_msgs 
  tf2 tf2_ros tf2_geometry_msgs octomap_msgs
)
target_link_libraries(nbv_explorer_3d_node ${OCTOMAP_LIBRARIES})

# ==========================================
# 2. Second Brain: Graph Explorer (Dijkstra)
# ==========================================
add_executable(graph_explorer_3d_node src/graph_explorer_3d.cpp)
target_include_directories(graph_explorer_3d_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(graph_explorer_3d_node 
  rclcpp std_msgs geometry_msgs nav_msgs 
  tf2 tf2_ros tf2_geometry_msgs octomap_msgs
)
target_link_libraries(graph_explorer_3d_node ${OCTOMAP_LIBRARIES})

# ==========================================
# 3. Install BOTH Executables
# ==========================================
install(TARGETS
  frontier_detector_node
  nbv_explorer_3d_node
  graph_explorer_3d_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament_package() MUST BE THE LAST LINE
ament_package()