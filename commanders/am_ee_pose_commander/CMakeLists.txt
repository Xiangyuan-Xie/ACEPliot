cmake_minimum_required(VERSION 3.16)
project(am_ee_pose_commander)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(policy_inference REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(am_ee_pose_commander_core
  src/arm_sync.cpp
  src/offboard_conversion.cpp
  src/policy_contract.cpp
  src/vehicle_state.cpp
  src/x500_arm_kinematics.cpp
)
target_include_directories(am_ee_pose_commander_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(am_ee_pose_commander_core Eigen3::Eigen)
ament_target_dependencies(am_ee_pose_commander_core
  px4_msgs
  px4_ros2_cpp
)

add_executable(am_ee_pose_commander_node src/am_ee_pose_commander_node.cpp)
target_link_libraries(am_ee_pose_commander_node
  am_ee_pose_commander_core
)
ament_target_dependencies(am_ee_pose_commander_node
  geometry_msgs
  policy_inference
  px4_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS am_ee_pose_commander_core am_ee_pose_commander_node
  EXPORT am_ee_pose_commander_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(am_ee_pose_commander_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3 px4_msgs px4_ros2_cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_xmllint
  )
  ament_lint_auto_find_test_dependencies()

  foreach(test_name IN ITEMS arm_sync policy_contract vehicle_state x500_arm_kinematics)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    if(TARGET test_${test_name})
      target_link_libraries(test_${test_name} am_ee_pose_commander_core)
    endif()
  endforeach()
endif()

ament_package()
