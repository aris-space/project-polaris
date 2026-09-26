cmake_minimum_required(VERSION 3.12.2)
project(orca_sim_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  PROGRAMS
  scripts/odom_to_tf.py
  scripts/current_vector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE cfg launch)
