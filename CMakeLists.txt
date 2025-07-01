cmake_minimum_required(VERSION 3.8)
project(localization_filters)

# Enable C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# === Kalman Filter Node ===
add_executable(kf_node src/kalman_filter.cpp)
ament_target_dependencies(kf_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_filters
)

# === Extended Kalman Filter Node ===
add_executable(ekf_node src/ekf.cpp)
ament_target_dependencies(ekf_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_filters
)

# === Particle Filter Node ===
add_executable(pf_node src/particle_filter.cpp)
ament_target_dependencies(pf_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_filters
)

# === Multi-Filter Pose Logger Node ===
add_executable(pose_logger src/pose_logger.cpp)
ament_target_dependencies(pose_logger
  rclcpp
  geometry_msgs
  nav_msgs
)

# === Navigator Node ===
add_executable(navigator src/navigator.cpp)
ament_target_dependencies(navigator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
)

# === Circle Driver Node ===
add_executable(drive_circle src/drive_circle.cpp)
ament_target_dependencies(drive_circle
  rclcpp
  geometry_msgs
)

# === Circle Rectangle Node ===
add_executable(drive_rectangle src/drive_rectangle.cpp)
ament_target_dependencies(drive_rectangle
  rclcpp
  geometry_msgs
)

# === cmd_velocity watchdog ===
add_executable(cmd_vel_watchdog src/cmd_vel_watchdog.cpp)
ament_target_dependencies(cmd_vel_watchdog
  rclcpp
  geometry_msgs
)

# === Install All Binaries ===
install(TARGETS
  kf_node
  ekf_node
  pf_node
  pose_logger
  navigator
  drive_circle
  drive_rectangle
  cmd_vel_watchdog
  DESTINATION lib/${PROJECT_NAME}
)

# === Install Headers (Optional) ===
install(DIRECTORY
  include/
  DESTINATION include/
)

# === Install Launch Files ===
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

# === Install Config Files (YAML, maps, etc.) ===
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()