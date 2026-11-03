cmake_minimum_required(VERSION 3.8)
project(robot_features)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === FIND DEPENDENCIES ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tr_robot_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === EXECUTABLES ===

# 1. Keyboard Teleop
add_executable(keyboard_control src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_control rclcpp geometry_msgs nav_msgs std_msgs)

# 2. Foxglove Global Node
add_executable(foxglove_counter src/foxglove_global_node.cpp)
ament_target_dependencies(foxglove_counter rclcpp geometry_msgs nav_msgs std_msgs)

# 3. Object Detector (Vision)
add_executable(object_detector src/object_detector.cpp)
ament_target_dependencies(object_detector rclcpp sensor_msgs cv_bridge)
target_link_libraries(object_detector ${OpenCV_LIBS})

# 4. Semi Auto PID Controller
add_executable(semi_auto_controller src/semi_auto_controller.cpp)
ament_target_dependencies(semi_auto_controller rclcpp geometry_msgs nav_msgs std_msgs)

# 5. TR Planner (Action Server)
add_executable(tr_planner src/tr_planner.cpp)
ament_target_dependencies(tr_planner
  rclcpp
  rclcpp_action
  tr_robot_interfaces
  geometry_msgs
  nav_msgs
  std_msgs
)

# === INSTALL ===
install(TARGETS
    keyboard_control
    foxglove_counter
    object_detector
    semi_auto_controller
    tr_planner
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  control_ui/index.html
  DESTINATION share/${PROJECT_NAME}
)

ament_package()