cmake_minimum_required(VERSION 3.8)
project(ros2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf launch rviz models worlds photos/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
