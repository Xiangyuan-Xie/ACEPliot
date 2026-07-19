cmake_minimum_required(VERSION 3.5)
project(px4_velocity_commander)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

include_directories(include)

add_library(px4_velocity_commander
  src/acesim_odometry_measurement.cpp
  src/clock_fallback.cpp
  src/mocap_velocity_estimator.cpp
  src/offboard_conversion.cpp
  src/velocity_profile.cpp
)

ament_target_dependencies(px4_velocity_commander
  nav_msgs
  px4_msgs
)

add_executable(px4_velocity_commander_node
  src/velocity_commander_node.cpp
)

ament_target_dependencies(px4_velocity_commander_node
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  rosgraph_msgs
)

target_link_libraries(px4_velocity_commander_node
  px4_velocity_commander
)

install(
  TARGETS
    px4_velocity_commander
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS
    px4_velocity_commander_node
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

  ament_add_gtest(test_velocity_profile test/test_velocity_profile.cpp)
  if(TARGET test_velocity_profile)
    target_link_libraries(test_velocity_profile px4_velocity_commander)
  endif()

  ament_add_gtest(test_mocap_velocity_estimator test/test_mocap_velocity_estimator.cpp)
  if(TARGET test_mocap_velocity_estimator)
    target_link_libraries(test_mocap_velocity_estimator px4_velocity_commander)
  endif()

  ament_add_gtest(test_offboard_conversion test/test_offboard_conversion.cpp)
  if(TARGET test_offboard_conversion)
    target_link_libraries(test_offboard_conversion px4_velocity_commander)
    ament_target_dependencies(test_offboard_conversion px4_msgs)
  endif()

  ament_add_gtest(test_clock_fallback test/test_clock_fallback.cpp)
  if(TARGET test_clock_fallback)
    target_link_libraries(test_clock_fallback px4_velocity_commander)
  endif()

  ament_add_gtest(test_acesim_odometry_measurement test/test_acesim_odometry_measurement.cpp)
  if(TARGET test_acesim_odometry_measurement)
    target_link_libraries(test_acesim_odometry_measurement px4_velocity_commander)
    ament_target_dependencies(test_acesim_odometry_measurement nav_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(px4_velocity_commander)
ament_export_dependencies(geometry_msgs nav_msgs px4_msgs rclcpp rosgraph_msgs)

ament_package()
