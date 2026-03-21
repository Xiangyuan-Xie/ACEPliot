cmake_minimum_required(VERSION 3.5)
project(rl_mc_arm_position_mode)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rl_base_mode REQUIRED)
find_package(rl_mode_utils REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

add_library(rl_mc_arm_position_mode
  src/arm_robot_data.cpp
  src/rl_mc_arm_position_direct_actuators_mode.cpp
  src/rl_mc_arm_position_rates_thrust_mode.cpp
)

ament_target_dependencies(rl_mc_arm_position_mode
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  px4_msgs
  px4_ros2_cpp
  rl_base_mode
  rl_mode_utils
)

add_executable(rl_mc_arm_position_direct_actuators_node
  src/main_direct_actuators.cpp
)

ament_target_dependencies(rl_mc_arm_position_direct_actuators_node
  rclcpp
  px4_ros2_cpp
)

target_link_libraries(rl_mc_arm_position_direct_actuators_node
  rl_mc_arm_position_mode
)

set_target_properties(
  rl_mc_arm_position_direct_actuators_node PROPERTIES
  OUTPUT_NAME rl_mc_arm_position_direct_actuators_mode)

add_executable(rl_mc_arm_position_rates_thrust_node
  src/main_rates_thrust.cpp
)

ament_target_dependencies(rl_mc_arm_position_rates_thrust_node
  rclcpp
  px4_ros2_cpp
)

target_link_libraries(rl_mc_arm_position_rates_thrust_node
  rl_mc_arm_position_mode
)

set_target_properties(
  rl_mc_arm_position_rates_thrust_node PROPERTIES
  OUTPUT_NAME rl_mc_arm_position_rates_thrust_mode)

ament_export_targets(rl_mc_arm_position_mode HAS_LIBRARY_TARGET)
install(
  TARGETS
    rl_mc_arm_position_mode
  EXPORT rl_mc_arm_position_mode
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  TARGETS
    rl_mc_arm_position_direct_actuators_node
    rl_mc_arm_position_rates_thrust_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/weights")
  install(
    DIRECTORY weights
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  px4_msgs
  px4_ros2_cpp
  rl_base_mode
  rl_mode_utils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
