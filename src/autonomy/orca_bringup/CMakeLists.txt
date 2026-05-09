cmake_minimum_required(VERSION 3.12.2)
project(orca_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install scripts
install(
  PROGRAMS
  scripts/dump_rosout.py
  scripts/export_tracking_bag_csv.py
  scripts/load_wsg84_points_to_waypoints.py
  scripts/nav2_ready_wait.py
  scripts/pixhawk_ready_wait.py
  scripts/mission_runner.py
  scripts/current_vector_node.py
  scripts/odom_to_tf.py
  scripts/WSG84_mission_starter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE behavior_trees cfg launch missions params)