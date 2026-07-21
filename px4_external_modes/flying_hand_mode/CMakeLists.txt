cmake_minimum_required(VERSION 3.16)
project(flying_hand_mode)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Werror
    "-ffile-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=flying_hand_mode"
    "-fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=flying_hand_mode"
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(acados_vendor REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(flying_hand_runtime SHARED
  src/runtime/controller_worker.cpp
  src/runtime/external_mode.cpp
  src/runtime/wrench_setpoint_type.cpp
)
target_include_directories(flying_hand_runtime PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(flying_hand_runtime Eigen3::Eigen Threads::Threads)
ament_target_dependencies(flying_hand_runtime
  diagnostic_msgs
  geometry_msgs
  px4_msgs
  px4_ros2_cpp
  rclcpp
  sensor_msgs
  std_msgs
)

set(QUADROTOR_GENERATED_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/generated/flying_hand_quadrotor_solver")
file(GLOB QUADROTOR_GENERATED_MODEL_SOURCES CONFIGURE_DEPENDS
  "${QUADROTOR_GENERATED_DIR}/flying_hand_quadrotor_model/*.c"
  "${QUADROTOR_GENERATED_DIR}/flying_hand_quadrotor_cost/*.c"
  "${QUADROTOR_GENERATED_DIR}/flying_hand_quadrotor_constraints/*.c"
)
add_library(flying_hand_quadrotor_generated_solver STATIC
  "${QUADROTOR_GENERATED_DIR}/acados_solver_flying_hand_quadrotor.c"
  ${QUADROTOR_GENERATED_MODEL_SOURCES}
)
target_include_directories(flying_hand_quadrotor_generated_solver PUBLIC
  "${QUADROTOR_GENERATED_DIR}"
)
ament_target_dependencies(flying_hand_quadrotor_generated_solver acados_vendor)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(flying_hand_quadrotor_generated_solver PRIVATE -w)
endif()

add_library(flying_hand_quadrotor_controller STATIC
  src/quadrotor/arm_kinematics.cpp
  src/quadrotor/controller_core.cpp
  src/quadrotor/flying_hand_controller.cpp
  src/quadrotor/whole_body_solver.cpp
)
target_include_directories(flying_hand_quadrotor_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(flying_hand_quadrotor_controller
  Eigen3::Eigen
  flying_hand_runtime
  flying_hand_quadrotor_generated_solver
)
ament_target_dependencies(flying_hand_quadrotor_controller acados_vendor rclcpp)

set(FULLY_ACTUATED_GENERATED_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/generated/flying_hand_fully_actuated_solver")
file(GLOB FULLY_ACTUATED_GENERATED_MODEL_SOURCES CONFIGURE_DEPENDS
  "${FULLY_ACTUATED_GENERATED_DIR}/flying_hand_fully_actuated_model/*.c"
  "${FULLY_ACTUATED_GENERATED_DIR}/flying_hand_fully_actuated_cost/*.c"
  "${FULLY_ACTUATED_GENERATED_DIR}/flying_hand_fully_actuated_constraints/*.c"
)
add_library(flying_hand_fully_actuated_generated_solver STATIC
  "${FULLY_ACTUATED_GENERATED_DIR}/acados_solver_flying_hand_fully_actuated.c"
  ${FULLY_ACTUATED_GENERATED_MODEL_SOURCES}
)
target_include_directories(flying_hand_fully_actuated_generated_solver PUBLIC
  "${FULLY_ACTUATED_GENERATED_DIR}"
)
ament_target_dependencies(flying_hand_fully_actuated_generated_solver acados_vendor)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(flying_hand_fully_actuated_generated_solver PRIVATE -w)
endif()

add_library(flying_hand_fully_actuated_controller STATIC
  src/fully_actuated/dh_kinematics.cpp
  src/fully_actuated/flying_hand_fully_actuated_controller.cpp
  src/fully_actuated/fully_actuated_core.cpp
  src/fully_actuated/paper_l1_adaptive.cpp
  src/fully_actuated/whole_body_solver.cpp
)
target_include_directories(flying_hand_fully_actuated_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(flying_hand_fully_actuated_controller
  Eigen3::Eigen
  flying_hand_runtime
  flying_hand_fully_actuated_generated_solver
)
ament_target_dependencies(flying_hand_fully_actuated_controller acados_vendor rclcpp)

add_executable(flying_hand_quadrotor src/quadrotor/main.cpp)
target_link_libraries(flying_hand_quadrotor
  flying_hand_quadrotor_controller
  flying_hand_runtime
)
ament_target_dependencies(flying_hand_quadrotor px4_msgs px4_ros2_cpp rclcpp)

add_executable(flying_hand_fully_actuated src/fully_actuated/main.cpp)
target_link_libraries(flying_hand_fully_actuated
  flying_hand_fully_actuated_controller
  flying_hand_runtime
)
ament_target_dependencies(flying_hand_fully_actuated px4_msgs px4_ros2_cpp rclcpp)

install(
  TARGETS flying_hand_runtime
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS flying_hand_quadrotor flying_hand_fully_actuated
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch models docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  acados_vendor
  diagnostic_msgs
  eigen3_cmake_module
  Eigen3
  geometry_msgs
  px4_msgs
  px4_ros2_cpp
  rclcpp
  sensor_msgs
  std_msgs
  Threads
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE generated)
  ament_lint_auto_find_test_dependencies()
  ament_uncrustify(include src test)

  foreach(test_name IN ITEMS controller_worker external_mode)
    ament_add_gtest(
      test_runtime_${test_name}
      test/runtime/test_${test_name}.cpp
    )
    if(TARGET test_runtime_${test_name})
      target_link_libraries(test_runtime_${test_name} flying_hand_runtime)
    endif()
  endforeach()

  foreach(test_name IN ITEMS arm_kinematics controller_core whole_body_solver)
    ament_add_gtest(
      test_quadrotor_${test_name}
      test/quadrotor/test_${test_name}.cpp
    )
    if(TARGET test_quadrotor_${test_name})
      target_link_libraries(
        test_quadrotor_${test_name}
        flying_hand_quadrotor_controller
      )
    endif()
  endforeach()

  ament_add_gtest(test_quadrotor_control_model test/quadrotor/test_control_model.cpp)
  if(TARGET test_quadrotor_control_model)
    target_link_libraries(test_quadrotor_control_model flying_hand_quadrotor_controller)
    target_compile_definitions(test_quadrotor_control_model PRIVATE
      ACEPILOT_CONTROL_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/models/x500_arm2x_control.urdf"
    )
  endif()

  foreach(test_name IN ITEMS
      dh_kinematics
      fully_actuated_controller
      fully_actuated_core
      paper_l1_adaptive
      whole_body_solver
  )
    ament_add_gtest(
      test_fully_actuated_${test_name}
      test/fully_actuated/test_${test_name}.cpp
    )
    if(TARGET test_fully_actuated_${test_name})
      target_link_libraries(
        test_fully_actuated_${test_name}
        flying_hand_fully_actuated_controller
      )
    endif()
  endforeach()
endif()

ament_package()
