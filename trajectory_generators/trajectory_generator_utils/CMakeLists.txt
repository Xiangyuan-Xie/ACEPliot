cmake_minimum_required(VERSION 3.5)
project(trajectory_generator_utils)

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

include_directories(include ${Eigen3_INCLUDE_DIRS})

add_library(trajectory_generator_utils
  src/offboard_conversion.cpp
  src/state_provider.cpp
)

ament_target_dependencies(trajectory_generator_utils
  rclcpp
  nav_msgs
  px4_msgs
  px4_ros2_cpp
)

target_link_libraries(trajectory_generator_utils
  Eigen3::Eigen
)

ament_export_targets(trajectory_generator_utils HAS_LIBRARY_TARGET)
ament_export_libraries(trajectory_generator_utils)

install(
  TARGETS trajectory_generator_utils
  EXPORT trajectory_generator_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_xmllint)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_trajectory_generator_utils_offboard_conversion
    test/test_offboard_conversion.cpp
  )
  if(TARGET test_trajectory_generator_utils_offboard_conversion)
    target_link_libraries(
      test_trajectory_generator_utils_offboard_conversion
      trajectory_generator_utils
    )
    ament_target_dependencies(
      test_trajectory_generator_utils_offboard_conversion
      px4_msgs
      px4_ros2_cpp
      nav_msgs
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  nav_msgs
  px4_msgs
  px4_ros2_cpp
  eigen3_cmake_module
  Eigen3
)

ament_package()
