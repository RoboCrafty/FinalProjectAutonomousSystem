cmake_minimum_required(VERSION 3.8)
project(decision_making_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)               # <--- ADD THIS
find_package(tf2_geometry_msgs REQUIRED)

add_executable(state_machine_node src/state_machine.cpp)

ament_target_dependencies(state_machine_node 
  rclcpp 
  nav_msgs 
  geometry_msgs
  trajectory_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

install(TARGETS state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
