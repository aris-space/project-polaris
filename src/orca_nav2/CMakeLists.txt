cmake_minimum_required(VERSION 3.12.2)
project(orca_nav2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

# Just list the packages here. ament_auto_find_build_dependencies 
# will use this list + your package.xml to set up the include paths.
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)

ament_auto_find_build_dependencies()

# --- Controller ---
# Added tf2 dependencies here
ament_auto_add_library(pure_pursuit_controller_3d SHARED src/pure_pursuit_controller_3d.cpp)
target_include_directories(pure_pursuit_controller_3d PRIVATE include)
ament_target_dependencies(pure_pursuit_controller_3d 
  nav2_util nav2_core nav2_costmap_2d rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs)
target_compile_definitions(pure_pursuit_controller_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core pure_pursuit_controller_3d_plugin.xml)

# --- Planner ---
ament_auto_add_library(straight_line_planner_3d SHARED src/straight_line_planner_3d.cpp)
target_include_directories(straight_line_planner_3d PRIVATE include)
ament_target_dependencies(straight_line_planner_3d nav2_util nav2_core nav2_costmap_2d rclcpp nav_msgs)
target_compile_definitions(straight_line_planner_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core straight_line_planner_3d_plugin.xml)

# --- Goal Checker ---
ament_auto_add_library(goal_checker_3d SHARED src/goal_checker_3d.cpp)
target_include_directories(goal_checker_3d PRIVATE include)
ament_target_dependencies(goal_checker_3d nav2_util nav2_core nav2_costmap_2d rclcpp angles)
target_compile_definitions(goal_checker_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core goal_checker_3d_plugin.xml)

# --- Progress Checker ---
ament_auto_add_library(progress_checker_3d SHARED src/progress_checker_3d.cpp)
target_include_directories(progress_checker_3d PRIVATE include)
ament_target_dependencies(progress_checker_3d nav2_util nav2_core nav2_costmap_2d rclcpp)
target_compile_definitions(progress_checker_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core progress_checker_3d_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  FILES
  pure_pursuit_controller_3d_plugin.xml
  straight_line_planner_3d_plugin.xml
  goal_checker_3d_plugin.xml
  progress_checker_3d_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()