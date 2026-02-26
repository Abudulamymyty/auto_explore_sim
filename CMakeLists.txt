cmake_minimum_required(VERSION 3.8)
project(auto_explore_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds config launch rviz models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts (LiDAR SLAM + ORB-SLAM3 visual SLAM helpers)
install(PROGRAMS
  scripts/frontier_explorer.py
  scripts/orb_slam3_tf_bridge.py
  scripts/orb_slam3_map_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
