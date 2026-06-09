cmake_minimum_required(VERSION 3.8)
project(ros2_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs tf2_ros tf2 std_msgs sensor_msgs nav_msgs)

add_executable(noisy_controller src/noisy_controller.cpp)
ament_target_dependencies(noisy_controller rclcpp geometry_msgs tf2_ros tf2 sensor_msgs nav_msgs )

add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp geometry_msgs )

target_link_libraries(simple_controller Eigen3::Eigen)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_controller
  noisy_controller
  twist_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
