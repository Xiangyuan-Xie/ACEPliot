# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

if (NOT TARGET onnxruntime-unofficial::onnxruntime)
    add_library(onnxruntime-unofficial::onnxruntime SHARED IMPORTED)
    set_property(TARGET onnxruntime-unofficial::onnxruntime APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
    set_target_properties(onnxruntime-unofficial::onnxruntime PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
    )
    if(UNIX)
        # Use unversioned .so name from install step
        set(_ORT_SO "${_IMPORT_PREFIX}/lib/libonnxruntime.so")
        if(NOT EXISTS "${_ORT_SO}")
            # Some packages may ship versioned soname only
            file(GLOB _ORT_SOS "${_IMPORT_PREFIX}/lib/libonnxruntime.so.*")
            list(GET _ORT_SOS 0 _ORT_SO)
        endif()
        set_target_properties(onnxruntime-unofficial::onnxruntime PROPERTIES
            IMPORTED_LOCATION_RELEASE "${_ORT_SO}"
        )
    else()
        message(FATAL_ERROR "unsupported platform.")
    endif()
endif()

list(APPEND onnxruntime_vendor_TARGETS onnxruntime-unofficial::onnxruntime)
