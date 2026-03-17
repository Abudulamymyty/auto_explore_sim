cmake_minimum_required(VERSION 3.8)
project(auto_explore_sim)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(frontier_explorer
  src/frontier_explorer.cpp
  src/frontier_search.cpp
  src/gvd_map.cpp
)
target_compile_features(frontier_explorer PUBLIC cxx_std_17)
target_include_directories(frontier_explorer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  frontier_explorer
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2_msgs
  visualization_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  worlds config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python explorer script
install(PROGRAMS
  scripts/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  frontier_explorer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
