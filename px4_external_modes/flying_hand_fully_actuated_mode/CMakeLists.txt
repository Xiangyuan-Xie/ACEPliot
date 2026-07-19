cmake_minimum_required(VERSION 3.16)
project(flying_hand_fully_actuated_mode)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Werror
    "-ffile-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=flying_hand_fully_actuated_mode"
    "-fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=flying_hand_fully_actuated_mode"
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

set(GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated/flying_hand_fully_actuated_solver")
file(GLOB GENERATED_MODEL_SOURCES CONFIGURE_DEPENDS
  "${GENERATED_DIR}/flying_hand_fully_actuated_model/*.c"
  "${GENERATED_DIR}/flying_hand_fully_actuated_cost/*.c"
  "${GENERATED_DIR}/flying_hand_fully_actuated_constraints/*.c"
)
add_library(flying_hand_fully_actuated_generated_solver STATIC
  "${GENERATED_DIR}/acados_solver_flying_hand_fully_actuated.c"
  ${GENERATED_MODEL_SOURCES}
)
target_include_directories(flying_hand_fully_actuated_generated_solver PUBLIC "${GENERATED_DIR}")
ament_target_dependencies(flying_hand_fully_actuated_generated_solver acados_vendor)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(flying_hand_fully_actuated_generated_solver PRIVATE -w)
endif()

add_library(flying_hand_fully_actuated_core
  src/dh_kinematics.cpp
  src/flying_hand_fully_actuated_controller.cpp
  src/fully_actuated_core.cpp
  src/paper_l1_adaptive.cpp
  src/whole_body_solver.cpp
)
target_include_directories(flying_hand_fully_actuated_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(flying_hand_fully_actuated_core
  Eigen3::Eigen
  flying_hand_fully_actuated_generated_solver
)
ament_target_dependencies(flying_hand_fully_actuated_core
  acados_vendor
  flying_hand_control_common
  rclcpp
)

add_executable(flying_hand_fully_actuated_mode_node src/main.cpp)
target_link_libraries(flying_hand_fully_actuated_mode_node flying_hand_fully_actuated_core)
ament_target_dependencies(flying_hand_fully_actuated_mode_node
  flying_hand_control_common
  px4_msgs
  px4_ros2_cpp
  rclcpp
)
install(
  TARGETS flying_hand_fully_actuated_mode_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint ament_cmake_uncrustify)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE generated)
  ament_lint_auto_find_test_dependencies()
  ament_uncrustify(include src test)

  foreach(test_name IN ITEMS
      dh_kinematics
      fully_actuated_controller
      fully_actuated_core
      paper_l1_adaptive
      whole_body_solver
  )
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    if(TARGET test_${test_name})
      target_link_libraries(test_${test_name} flying_hand_fully_actuated_core)
    endif()
  endforeach()
endif()

ament_package()
