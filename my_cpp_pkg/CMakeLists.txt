cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_node src/robot_news_station.cpp)
ament_target_dependencies(robot_news_node rclcpp example_interfaces)


add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(add_two_int_serves src/add_two_int_serves.cpp)
ament_target_dependencies(add_two_int_serves rclcpp example_interfaces)

add_executable(add_two_int_client_no_oop src/add_two_int_client_no_oop.cpp)
ament_target_dependencies(add_two_int_client_no_oop rclcpp example_interfaces)

add_executable(add_two_int_client src/add_two_int_client.cpp)
ament_target_dependencies(add_two_int_client rclcpp example_interfaces)

add_executable(hardware_status_publisher src/hardware_status_publisher.cpp)
ament_target_dependencies(hardware_status_publisher rclcpp std_msgs my_robot_interfaces)

add_executable(battery_node src/battery_node.cpp)
ament_target_dependencies(battery_node rclcpp std_msgs my_robot_interfaces)

add_executable(led_pannel_node src/led_pannel_node.cpp)
ament_target_dependencies(led_pannel_node rclcpp std_msgs my_robot_interfaces)

add_executable(number_publisher_with_parameter src/number_publisher_with_parameter.cpp)
ament_target_dependencies(number_publisher_with_parameter rclcpp std_msgs example_interfaces)

add_executable(simple_turtlesim_kinematics src/simple_turtlesim_kinematics.cpp)
ament_target_dependencies(simple_turtlesim_kinematics rclcpp turtlesim)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics rclcpp geometry_msgs tf2_ros my_robot_interfaces tf2)

add_executable(simple_service_server src/simple_service_server.cpp)
ament_target_dependencies(simple_service_server rclcpp my_robot_interfaces)

add_executable(simple_service_client src/simple_service_client.cpp)
ament_target_dependencies(simple_service_client rclcpp my_robot_interfaces)

add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(simple_qos_publisher src/simple_qos_publisher.cpp)
ament_target_dependencies(simple_qos_publisher rclcpp std_msgs)

add_executable(simple_qos_subscriber src/simple_qos_subscriber.cpp)
ament_target_dependencies(simple_qos_subscriber rclcpp std_msgs)

target_include_directories(simple_turtlesim_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(simple_action_server SHARED src/simple_action_server.cpp)
target_include_directories(simple_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(simple_action_server
  PRIVATE "SIMPLE_ACTION_SERVER_CPP_BUILDING_DLL")

ament_target_dependencies(simple_action_server

  "ros2_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

rclcpp_components_register_node(simple_action_server 
  PLUGIN "my_cpp_pkg::SimpleActionServer" 
  EXECUTABLE simple_action_server_node
)


add_library(simple_action_client SHARED src/simple_action_client.cpp)
target_include_directories(simple_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(simple_action_client
  PRIVATE "SIMPLE_ACTION_CLIENT_CPP_BUILDING_DLL")

ament_target_dependencies(simple_action_client

  "ros2_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

rclcpp_components_register_node(simple_action_client
  PLUGIN "my_cpp_pkg::SimpleActionClient" 
  EXECUTABLE simple_action_client_node
)
install(
  DIRECTORY include
  DESTINATION include
)
install(TARGETS
  cpp_node
  robot_news_node
  smartphone
  number_publisher
  number_counter
  add_two_int_serves
  add_two_int_client_no_oop
  add_two_int_client
  hardware_status_publisher
  battery_node
  led_pannel_node
  number_publisher_with_parameter
  simple_turtlesim_kinematics
  simple_tf_kinematics
  simple_service_server
  simple_service_client
  simple_lifecycle_node
  simple_qos_subscriber
  simple_qos_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_action_server
  simple_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_package()
