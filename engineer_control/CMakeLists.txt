cmake_minimum_required(VERSION 3.8)
project(engineer_control)

# 2. 自动在build目录下生成compile_commands.json文件
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(custom_interfaces REQUIRED)


add_executable(engineer_control src/engineer_control.cpp)
add_executable(engineer_frame src/engineer_frame.cpp)
add_executable(key_control src/key_control.cpp)
add_executable(control_scheduling src/control_scheduling.cpp)


INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/engineer_control/
)


target_include_directories(engineer_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(engineer_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


ament_target_dependencies(
  engineer_control
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "moveit_visual_tools"
  "custom_interfaces"
)

ament_target_dependencies(
  engineer_frame
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "moveit_visual_tools"
)

ament_target_dependencies(
  key_control
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "moveit_visual_tools"
  "custom_interfaces"
)

ament_target_dependencies(
  control_scheduling
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "moveit_visual_tools"
  "custom_interfaces"
)


install(TARGETS engineer_control engineer_frame key_control control_scheduling
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
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
