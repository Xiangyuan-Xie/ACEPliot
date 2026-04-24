cmake_minimum_required(VERSION 3.5)
project(rc_manual_trajectory_mode)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(trajectory_generator_utils REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})

add_library(rc_manual_trajectory_mode
  src/rc_manual_generator.cpp
)

ament_target_dependencies(rc_manual_trajectory_mode
  rclcpp
  px4_msgs
  px4_ros2_cpp
  trajectory_generator_utils
)

target_link_libraries(rc_manual_trajectory_mode
  Eigen3::Eigen
)

add_executable(rc_manual_trajectory_mode_node
  src/rc_manual_trajectory_mode_node.cpp
)

ament_target_dependencies(rc_manual_trajectory_mode_node
  rclcpp
  px4_msgs
  px4_ros2_cpp
  trajectory_generator_utils
)

target_link_libraries(rc_manual_trajectory_mode_node
  rc_manual_trajectory_mode
)

install(
  TARGETS
    rc_manual_trajectory_mode
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS
    rc_manual_trajectory_mode_node
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(BUILD_TESTING)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_xmllint)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  px4_msgs
  px4_ros2_cpp
  trajectory_generator_utils
  eigen3_cmake_module
  Eigen3
)

ament_package()
