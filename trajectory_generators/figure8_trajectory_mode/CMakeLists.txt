cmake_minimum_required(VERSION 3.5)
project(figure8_trajectory_mode)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(trajectory_generator_utils REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})

add_library(figure8_trajectory_mode
  src/figure8_generator.cpp
)

ament_target_dependencies(figure8_trajectory_mode
  rclcpp
  nav_msgs
  px4_msgs
  px4_ros2_cpp
  trajectory_generator_utils
)

target_link_libraries(figure8_trajectory_mode
  Eigen3::Eigen
)

add_executable(figure8_position_mode_node
  src/figure8_position_mode_node.cpp
)

ament_target_dependencies(figure8_position_mode_node
  rclcpp
  nav_msgs
  px4_msgs
  px4_ros2_cpp
  trajectory_generator_utils
)

target_link_libraries(figure8_position_mode_node
  figure8_trajectory_mode
)

add_executable(figure8_velocity_mode_node
  src/figure8_velocity_mode_node.cpp
)

ament_target_dependencies(figure8_velocity_mode_node
  rclcpp
  nav_msgs
  px4_msgs
  px4_ros2_cpp
  trajectory_generator_utils
)

target_link_libraries(figure8_velocity_mode_node
  figure8_trajectory_mode
)

install(
  TARGETS
    figure8_trajectory_mode
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS
    figure8_position_mode_node
    figure8_velocity_mode_node
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

  ament_add_gtest(test_figure8_generator
    test/test_figure8_generator.cpp
  )
  if(TARGET test_figure8_generator)
    target_link_libraries(test_figure8_generator figure8_trajectory_mode)
    ament_target_dependencies(test_figure8_generator rclcpp nav_msgs trajectory_generator_utils)
  endif()

  ament_add_gtest(test_figure8_mode_utils
    test/test_figure8_mode_utils.cpp
  )
  if(TARGET test_figure8_mode_utils)
    target_include_directories(test_figure8_mode_utils PRIVATE src)
    ament_target_dependencies(test_figure8_mode_utils trajectory_generator_utils)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  nav_msgs
  px4_msgs
  px4_ros2_cpp
  trajectory_generator_utils
  eigen3_cmake_module
  Eigen3
)

ament_package()
