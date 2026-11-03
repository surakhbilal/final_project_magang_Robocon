cmake_minimum_required(VERSION 3.8)
project(br_robot_features)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === FIND DEPENDENCIES ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# TAMBAHAN: Dependency untuk Action dan Interface
find_package(rclcpp_action REQUIRED)
find_package(br_robot_interfaces REQUIRED)

# === EXECUTABLES ===
add_executable(keyboard_control src/br_keyboard_teleop.cpp)
ament_target_dependencies(keyboard_control
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

add_executable(object_detector src/br_object_detector.cpp)
ament_target_dependencies(object_detector rclcpp sensor_msgs cv_bridge OpenCV)

# TAMBAHAN: Executable untuk Semi Auto (Sopir)
add_executable(br_semi_auto src/br_semi_auto.cpp)
ament_target_dependencies(br_semi_auto
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# TAMBAHAN: Executable untuk Planner (Mandor)
add_executable(br_planner src/br_planner.cpp)
ament_target_dependencies(br_planner
  rclcpp
  rclcpp_action
  br_robot_interfaces
  geometry_msgs
  nav_msgs
  std_msgs
)

# === INSTALL ===
install(TARGETS
  keyboard_control
  object_detector
  br_semi_auto
  br_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()