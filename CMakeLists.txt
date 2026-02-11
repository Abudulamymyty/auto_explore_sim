cmake_minimum_required(VERSION 3.8)
project(auto_explore_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
