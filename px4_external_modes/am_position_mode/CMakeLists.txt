cmake_minimum_required(VERSION 3.8)
project(am_position_mode)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(policy_inference REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(am_position_mode
  src/am_position_mode.cpp
  src/am_position_offboard_reference.cpp
  src/arm_robot_data.cpp
  src/flight_logger.cpp
  src/math.cpp
  src/observation_buffer.cpp
  src/robot_data.cpp
)
target_include_directories(am_position_mode
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(am_position_mode PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
target_link_libraries(am_position_mode Eigen3::Eigen)
ament_target_dependencies(am_position_mode
  geometry_msgs
  nav_msgs
  policy_inference
  px4_msgs
  px4_ros2_cpp
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
)

add_executable(am_position_mode_node src/main.cpp)
target_link_libraries(am_position_mode_node am_position_mode)
ament_target_dependencies(am_position_mode_node rclcpp px4_ros2_cpp)
set_target_properties(am_position_mode_node PROPERTIES OUTPUT_NAME am_position_mode)

install(
  TARGETS am_position_mode am_position_mode_node
  EXPORT am_position_mode_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/weights")
  install(DIRECTORY weights DESTINATION share/${PROJECT_NAME})
endif()

ament_export_targets(am_position_mode_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  nav_msgs
  policy_inference
  px4_msgs
  px4_ros2_cpp
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_am_position_mode test/test_am_position_mode.cpp)
  if(TARGET test_am_position_mode)
    target_link_libraries(test_am_position_mode am_position_mode)
  endif()

  ament_add_gtest(
    test_am_position_offboard_reference
    test/test_am_position_offboard_reference.cpp
  )
  if(TARGET test_am_position_offboard_reference)
    target_link_libraries(test_am_position_offboard_reference am_position_mode)
    ament_target_dependencies(
      test_am_position_offboard_reference rclcpp px4_msgs px4_ros2_cpp)
  endif()
endif()

ament_package()
