cmake_minimum_required(VERSION 3.8)
project(robot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  pluginlib
)

include_directories(include)

add_library(Dijkstra_planning  SHARED src/Dijkstra_planning.cpp)
ament_target_dependencies(Dijkstra_planning ${dependencies}) 

add_library(AStar_planning SHARED  src/AStar_planning.cpp)
ament_target_dependencies(AStar_planning ${dependencies})

pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  Dijkstra_planning
  AStar_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES global_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  Dijkstra_planning
  AStar_planning
)
ament_export_dependencies(${dependencies})
ament_package()
