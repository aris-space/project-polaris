cmake_minimum_required(VERSION 3.12.2)
project(orca_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Nodes used by the simulation-only bringup path.
ament_auto_add_executable(odom_to_path_node src/odom_to_path_node.cpp)
ament_auto_add_executable(pose_to_path_node src/pose_to_path_node.cpp)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install scripts
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/test_base.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)