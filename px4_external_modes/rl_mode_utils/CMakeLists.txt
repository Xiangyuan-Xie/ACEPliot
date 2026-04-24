cmake_minimum_required(VERSION 3.5)
project(rl_mode_utils)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

set(HEADER_FILES
    include/inference/infer_backend.hpp
    include/rnn_state_manager.hpp
    include/flight_logger.hpp
    include/flight_logger.tpp
    include/inference/onnx_ort_backend.hpp
    include/math.hpp
    include/observation_buffer.hpp
)

add_library(rl_mode_utils
    ${HEADER_FILES}
    src/rnn_state_manager.cpp
    src/flight_logger.cpp
    src/inference/onnx_ort_backend.cpp
    src/observation_buffer.cpp
    src/math.cpp
)

ament_target_dependencies(rl_mode_utils
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_msgs
  cv_bridge
  rosbag2_cpp
  rosbag2_storage
)

target_compile_definitions(rl_mode_utils PRIVATE USE_ORT_ALLOCATED_NAME_API=1)

target_link_libraries(rl_mode_utils
    Eigen3::Eigen
    onnxruntime-unofficial::onnxruntime
)

ament_export_targets(rl_mode_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    eigen3_cmake_module
    Eigen3
    onnxruntime_vendor
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
    tf2_msgs
    cv_bridge
    rosbag2_cpp
    rosbag2_storage
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS rl_mode_utils
    EXPORT rl_mode_utils
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
