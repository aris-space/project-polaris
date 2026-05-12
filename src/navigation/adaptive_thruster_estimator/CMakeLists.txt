cmake_minimum_required(VERSION 3.8)
project(adaptive_thruster_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp     REQUIRED)
find_package(nav_msgs   REQUIRED)
find_package(std_msgs   REQUIRED)
find_package(Eigen3     REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(adaptive_thruster_estimator
  src/adaptive_thruster_estimator.cpp)

target_include_directories(adaptive_thruster_estimator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(adaptive_thruster_estimator SYSTEM
  PRIVATE ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(adaptive_thruster_estimator
  rclcpp nav_msgs std_msgs
  # std_msgs covers both Int16MultiArray and Float64MultiArray
  )

install(TARGETS adaptive_thruster_estimator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
