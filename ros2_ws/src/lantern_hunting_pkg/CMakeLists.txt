cmake_minimum_required(VERSION 3.8)
project(lantern_hunting_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Updated to use hunting_node.cpp and output an executable named hunting_node
add_executable(hunting_node src/hunting_node.cpp)

ament_target_dependencies(hunting_node
  rclcpp
  geometry_msgs
  nav_msgs
  trajectory_msgs
)

# Updated to install the newly named executable
install(TARGETS hunting_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
