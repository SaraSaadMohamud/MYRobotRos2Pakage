cmake_minimum_required(VERSION 3.8)
project(robot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)                    
find_package(tf2_geometry_msgs REQUIRED)

# Important: Tell CMake where to find your headers during build
include_directories(include)

add_library(pd_motion_planner SHARED src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner rclcpp geometry_msgs nav_msgs nav2_core nav2_util nav2_costmap_2d tf2_ros tf2 tf2_geometry_msgs pluginlib)

add_library(PurePursuit SHARED src/PurePursuit.cpp)
ament_target_dependencies(PurePursuit rclcpp geometry_msgs nav_msgs nav2_core nav2_util nav2_costmap_2d tf2_ros tf2 tf2_geometry_msgs pluginlib)

pluginlib_export_plugin_description_file(nav2_core motion_planner_plugins.xml)


install(TARGETS
  pd_motion_planner
  PurePursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES motion_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
# Install headers (this makes the "robot_motion/..." include style work)
install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs nav2_core nav2_util nav2_costmap_2d tf2_ros tf2 tf2_geometry_msgs pluginlib)
ament_export_libraries(pd_motion_planner  PurePursuit)
ament_package()