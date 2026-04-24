cmake_minimum_required(VERSION 3.5)
project(px4_state_converter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
set(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})

# odom_from_px4_node
add_executable(odom_from_px4_node src/odom_from_px4_node.cpp)
ament_target_dependencies(odom_from_px4_node rclcpp px4_msgs px4_ros2_cpp nav_msgs tf2_ros geometry_msgs)

# gt_odom_from_px4_node
add_executable(gt_odom_from_px4_node src/gt_odom_from_px4_node.cpp)
ament_target_dependencies(gt_odom_from_px4_node rclcpp px4_msgs px4_ros2_cpp nav_msgs tf2_ros geometry_msgs message_filters)

# imu_from_px4_node
add_executable(imu_from_px4_node src/imu_from_px4_node.cpp)
ament_target_dependencies(imu_from_px4_node rclcpp px4_msgs px4_ros2_cpp sensor_msgs)

# odom_to_px4_node
add_executable(odom_to_px4_node src/odom_to_px4_node.cpp)
ament_target_dependencies(odom_to_px4_node rclcpp geometry_msgs nav_msgs tf2 Eigen3 px4_ros2_cpp)

# gt_odom_to_px4_node
add_executable(gt_odom_to_px4_node src/gt_odom_to_px4_node.cpp)
ament_target_dependencies(gt_odom_to_px4_node rclcpp geometry_msgs message_filters px4_ros2_cpp px4_msgs)

install(TARGETS
  odom_from_px4_node
  gt_odom_from_px4_node
  imu_from_px4_node
  odom_to_px4_node
  gt_odom_to_px4_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
