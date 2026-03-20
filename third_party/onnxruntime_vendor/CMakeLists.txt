# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)
project(onnxruntime_vendor)

find_package(ament_cmake REQUIRED)

set(ORT_VERSION "1.18.0" CACHE STRING "ONNX Runtime version to fetch from NuGet")
set(ORT_SHA512 "" CACHE STRING "Optional SHA512 checksum for the NuGet package")
set(ORT_PACKAGE_ID "Microsoft.ML.OnnxRuntime" CACHE STRING
    "NuGet package ID (e.g., Microsoft.ML.OnnxRuntime or Microsoft.ML.OnnxRuntime.Gpu)")
set(ORT_PACKAGE_URL "" CACHE STRING "Override full package URL; if set, supersedes package id/version")

if(ORT_PACKAGE_URL)
  set(PACKAGE_URL "${ORT_PACKAGE_URL}")
else()
  set(PACKAGE_URL "https://www.nuget.org/api/v2/package/${ORT_PACKAGE_ID}/${ORT_VERSION}")
endif()

# Detect target runtime directory by architecture (supports x86_64 and aarch64/arm64)
if(NOT ORT_RUNTIME_DIR)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(ORT_RUNTIME_DIR "linux-x64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(ORT_RUNTIME_DIR "linux-arm64")
  else()
    message(WARNING
        "Unknown architecture '${CMAKE_SYSTEM_PROCESSOR}'. Defaulting to linux-x64. You can override with -DORT_RUNTIME_DIR=...")
    set(ORT_RUNTIME_DIR "linux-x64")
  endif()
endif()

# Cache under .cache folder next to this CMakeLists.txt
set(ORT_CACHE_DIR "${CMAKE_CURRENT_LIST_DIR}/.cache")
file(MAKE_DIRECTORY "${ORT_CACHE_DIR}")
set(ORT_NUGET_FILE "${ORT_CACHE_DIR}/onnxruntime-${ORT_VERSION}.nuget")

# Download only if not already present
set(_ort_need_download TRUE)
if(EXISTS "${ORT_NUGET_FILE}")
  file(SIZE "${ORT_NUGET_FILE}" _ort_existing_size)
  if(_ort_existing_size GREATER 0)
    set(_ort_need_download FALSE)
  endif()
endif()

if(_ort_need_download)
  string(TOLOWER "${ORT_PACKAGE_ID}" _ort_package_id_lower)
  set(_ort_download_urls "${PACKAGE_URL}")
  list(APPEND _ort_download_urls
      "https://api.nuget.org/v3-flatcontainer/${_ort_package_id_lower}/${ORT_VERSION}/${_ort_package_id_lower}.${ORT_VERSION}.nupkg")
  file(REMOVE "${ORT_NUGET_FILE}")
  set(_ort_download_ok FALSE)
  set(_ort_download_error "")
  foreach(_ort_url ${_ort_download_urls})
    file(REMOVE "${ORT_NUGET_FILE}.tmp")
    if(ORT_SHA512)
      file(DOWNLOAD
          ${_ort_url}
          "${ORT_NUGET_FILE}.tmp"
          EXPECTED_HASH SHA512=${ORT_SHA512}
          SHOW_PROGRESS
          STATUS _ort_download_status
          TIMEOUT 1800
          INACTIVITY_TIMEOUT 120
      )
    else()
      file(DOWNLOAD
          ${_ort_url}
          "${ORT_NUGET_FILE}.tmp"
          SHOW_PROGRESS
          STATUS _ort_download_status
          TIMEOUT 1800
          INACTIVITY_TIMEOUT 120
      )
    endif()

    list(GET _ort_download_status 0 _ort_download_code)
    list(GET _ort_download_status 1 _ort_download_message)
    if(_ort_download_code EQUAL 0 AND EXISTS "${ORT_NUGET_FILE}.tmp")
      file(SIZE "${ORT_NUGET_FILE}.tmp" _ort_downloaded_size)
      if(_ort_downloaded_size GREATER 0)
        file(RENAME "${ORT_NUGET_FILE}.tmp" "${ORT_NUGET_FILE}")
        set(_ort_download_ok TRUE)
        break()
      endif()
      set(_ort_download_error "Downloaded file is empty from ${_ort_url}")
    else()
      set(_ort_download_error "${_ort_download_message} (${_ort_url})")
    endif()
  endforeach()

  if(NOT _ort_download_ok)
    message(FATAL_ERROR "Failed to download ONNX Runtime package: ${_ort_download_error}")
  endif()
endif()

# Extract into build tree (not polluting source) but from cached nuget
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime")

add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/.extract_done"
    COMMAND ${CMAKE_COMMAND} -E tar xf "${ORT_NUGET_FILE}"
    COMMAND ${CMAKE_COMMAND} -E touch "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/.extract_done"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime"
    VERBATIM
)
add_custom_target(ExtractOnnxruntime ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/.extract_done")

install(CODE "
  set(_ort_include_dir_1 \"${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/build/native/include\")
  set(_ort_include_dir_2 \"${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/include\")
  if(EXISTS \"\${_ort_include_dir_1}\")
    file(MAKE_DIRECTORY \"${CMAKE_INSTALL_PREFIX}/include\")
    file(COPY \"\${_ort_include_dir_1}/\" DESTINATION \"${CMAKE_INSTALL_PREFIX}/include\")
  elseif(EXISTS \"\${_ort_include_dir_2}\")
    file(MAKE_DIRECTORY \"${CMAKE_INSTALL_PREFIX}/include\")
    file(COPY \"\${_ort_include_dir_2}/\" DESTINATION \"${CMAKE_INSTALL_PREFIX}/include\")
  else()
    message(FATAL_ERROR \"ONNX Runtime include directory not found\")
  endif()
")

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/runtimes/${ORT_RUNTIME_DIR}/native/libonnxruntime.so
    DESTINATION lib
)
if(UNIX)
  install(CODE "
    execute_process(
      COMMAND \"${CMAKE_COMMAND}\" -E create_symlink libonnxruntime.so libonnxruntime.so.${ORT_VERSION}
      WORKING_DIRECTORY \"${CMAKE_INSTALL_PREFIX}/lib\")"
)
endif()

# Optionally install GPU provider libs if they exist (Jetson/CUDA capable builds)
set(_ORT_CUDA_PROVIDER "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/runtimes/${ORT_RUNTIME_DIR}/native/libonnxruntime_providers_cuda.so")
set(_ORT_TRT_PROVIDER  "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/runtimes/${ORT_RUNTIME_DIR}/native/libonnxruntime_providers_tensorrt.so")
set(_ORT_PROV_SHARED   "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/runtimes/${ORT_RUNTIME_DIR}/native/libonnxruntime_providers_shared.so")
foreach(_prov ${_ORT_CUDA_PROVIDER} ${_ORT_TRT_PROVIDER} ${_ORT_PROV_SHARED})
  if(EXISTS "${_prov}")
    install(FILES "${_prov}" DESTINATION lib)
  endif()
endforeach()

# Handle license file names across ORT versions
set(_ORT_LICENSE_TXT "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/LICENSE.txt")
set(_ORT_LICENSE     "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/LICENSE")
if(EXISTS "${_ORT_LICENSE_TXT}")
  install(FILES "${_ORT_LICENSE_TXT}" DESTINATION share/${PROJECT_NAME})
elseif(EXISTS "${_ORT_LICENSE}")
  install(FILES "${_ORT_LICENSE}" DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/ThirdPartyNotices.txt")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/ThirdPartyNotices.txt" DESTINATION share/${PROJECT_NAME})
endif()

ament_package(
  CONFIG_EXTRAS "src/onnxruntime_vendor-extras.cmake"
)
