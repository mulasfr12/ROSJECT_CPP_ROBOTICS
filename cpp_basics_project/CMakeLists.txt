cmake_minimum_required(VERSION 3.8)
project(cpp_basics_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# include header files
include_directories(include)

# add executables
add_executable(robot_interface src/robot_interface_class.cpp src/robot_interface_main.cpp)
add_executable(robot_control_noclass src/robot_interface_class.cpp src/robot_control_noclass.cpp)
add_executable(robot_control_classed src/robot_interface_class.cpp src/robot_control_classed.cpp)

# link dependencies
ament_target_dependencies(robot_interface rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
ament_target_dependencies(robot_control_noclass rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
ament_target_dependencies(robot_control_classed rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

# install executables
install(TARGETS
  robot_interface
  robot_control_noclass
  robot_control_classed
  DESTINATION lib/${PROJECT_NAME}
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

# End of File
