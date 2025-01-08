cmake_minimum_required(VERSION 3.8)
project(control_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_actions REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories:
include_directories(include)

# Add executable:
add_executable(
  move_robot
  src/move_gripper.cpp
  src/move_linear.cpp
  src/move_joints.cpp
  src/move_xyzw.cpp
  src/move_robot.cpp
)
# Install executable:
install(TARGETS
  move_robot
  DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(
  move_robot
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  rclcpp_action
  control_actions
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
