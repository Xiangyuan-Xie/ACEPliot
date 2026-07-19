cmake_minimum_required(VERSION 3.5)
project(arm_trajectory_commander)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(arm_trajectory_commander
  src/arm_trajectory.cpp
  src/clock_fallback.cpp
  src/vehicle_position_metrics.cpp
)

add_executable(arm_trajectory_commander_node
  src/arm_trajectory_commander_node.cpp
)

ament_target_dependencies(arm_trajectory_commander_node
  geometry_msgs
  nav_msgs
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(arm_trajectory_commander_node
  arm_trajectory_commander
)

install(
  TARGETS
    arm_trajectory_commander
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS
    arm_trajectory_commander_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_xmllint)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_arm_trajectory test/test_arm_trajectory.cpp)
  if(TARGET test_arm_trajectory)
    target_link_libraries(test_arm_trajectory arm_trajectory_commander)
  endif()

  ament_add_gtest(test_clock_fallback test/test_clock_fallback.cpp)
  if(TARGET test_clock_fallback)
    target_link_libraries(test_clock_fallback arm_trajectory_commander)
  endif()

  ament_add_gtest(test_vehicle_position_metrics test/test_vehicle_position_metrics.cpp)
  if(TARGET test_vehicle_position_metrics)
    target_link_libraries(test_vehicle_position_metrics arm_trajectory_commander)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(arm_trajectory_commander)
ament_export_dependencies(geometry_msgs nav_msgs rclcpp rosgraph_msgs sensor_msgs std_msgs)

ament_package()
