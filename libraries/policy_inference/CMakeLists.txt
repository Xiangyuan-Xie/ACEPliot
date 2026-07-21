cmake_minimum_required(VERSION 3.16)
project(policy_inference)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(onnxruntime_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/onnx_policy.cpp
  src/recurrent_state.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} PRIVATE onnxruntime-unofficial::onnxruntime)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(onnxruntime_vendor)

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

  ament_add_gtest(test_recurrent_state test/test_recurrent_state.cpp)
  if(TARGET test_recurrent_state)
    target_link_libraries(test_recurrent_state ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_onnx_policy test/test_onnx_policy.cpp)
  if(TARGET test_onnx_policy)
    target_link_libraries(test_onnx_policy ${PROJECT_NAME})
  endif()
endif()

ament_package()
