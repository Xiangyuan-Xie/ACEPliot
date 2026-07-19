cmake_minimum_required(VERSION 3.16)
project(flying_hand_quadrotor_mode)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Werror
    "-ffile-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=flying_hand_quadrotor_mode"
    "-fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=flying_hand_quadrotor_mode"
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(acados_vendor REQUIRED)
find_package(flying_hand_control_common REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rclcpp REQUIRED)

set(GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated/flying_hand_quadrotor_solver")
file(GLOB GENERATED_MODEL_SOURCES CONFIGURE_DEPENDS
  "${GENERATED_DIR}/flying_hand_quadrotor_model/*.c"
  "${GENERATED_DIR}/flying_hand_quadrotor_cost/*.c"
  "${GENERATED_DIR}/flying_hand_quadrotor_constraints/*.c"
)
add_library(flying_hand_quadrotor_generated_solver STATIC
  "${GENERATED_DIR}/acados_solver_flying_hand_quadrotor.c"
  ${GENERATED_MODEL_SOURCES}
)
target_include_directories(flying_hand_quadrotor_generated_solver PUBLIC
  "${GENERATED_DIR}"
)
ament_target_dependencies(flying_hand_quadrotor_generated_solver acados_vendor)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(flying_hand_quadrotor_generated_solver PRIVATE -w)
endif()

add_library(flying_hand_quadrotor_control
  src/arm_kinematics.cpp
  src/controller_core.cpp
  src/flying_hand_controller.cpp
  src/whole_body_solver.cpp
)
target_include_directories(flying_hand_quadrotor_control PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(flying_hand_quadrotor_control
  Eigen3::Eigen
  flying_hand_quadrotor_generated_solver
)
ament_target_dependencies(flying_hand_quadrotor_control
  acados_vendor
  flying_hand_control_common
  rclcpp
)

add_executable(flying_hand_quadrotor_mode_node src/main.cpp)
target_link_libraries(flying_hand_quadrotor_mode_node flying_hand_quadrotor_control)
ament_target_dependencies(flying_hand_quadrotor_mode_node
  flying_hand_control_common
  px4_msgs
  px4_ros2_cpp
  rclcpp
)
install(TARGETS flying_hand_quadrotor_mode_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

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

  foreach(test_name IN ITEMS arm_kinematics controller_core whole_body_solver control_model)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    if(TARGET test_${test_name})
      target_link_libraries(test_${test_name} flying_hand_quadrotor_control)
      target_compile_definitions(test_${test_name} PRIVATE
        ACEPILOT_CONTROL_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/models/x500_arm2x_control.urdf"
      )
    endif()
  endforeach()

endif()

ament_package()
