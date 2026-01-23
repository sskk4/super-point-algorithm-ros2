cmake_minimum_required(VERSION 3.8)
project(vertex_detector)

# C++ Standard (17 dla parallel algorithms)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release mode optimizations
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Aggressive optimizations for Jetson Orin
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -mtune=native -flto -ffast-math")

# Intel TBB for parallel execution (jeśli dostępne)
find_package(TBB QUIET)
if(TBB_FOUND)
  message(STATUS "Intel TBB found - enabling parallel algorithms")
  add_compile_definitions(USE_TBB)
else()
  message(STATUS "Intel TBB not found - install with: sudo apt install libtbb-dev")
endif()

# ============================================================================
# DEPENDENCIES
# ============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# CUDA (for TensorRT)
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

# TensorRT - ścieżki dla Jetson Orin
set(TENSORRT_ROOT /usr/src/tensorrt)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS ${TENSORRT_ROOT}/include
  PATH_SUFFIXES include)
  
find_library(TENSORRT_LIBRARY nvinfer
  HINTS ${TENSORRT_ROOT}/lib
  PATH_SUFFIXES lib lib64)

find_library(TENSORRT_ONNX_PARSER nvonnxparser
  HINTS ${TENSORRT_ROOT}/lib
  PATH_SUFFIXES lib lib64)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR "TensorRT not found! Install with: sudo apt install libnvinfer-dev libnvonnxparsers-dev")
endif()

message(STATUS "TensorRT found:")
message(STATUS "  Include: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "  Library: ${TENSORRT_LIBRARY}")

include_directories(
  include
  ${TENSORRT_INCLUDE_DIR}
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  cv_bridge
)

# ============================================================================
# LIBRARY - Composable Node
# ============================================================================

add_library(vertex_detector_component SHARED
  src/vertex_detector_node.cpp
  src/image_processor.cpp
  src/nms_filter.cpp
)

ament_target_dependencies(vertex_detector_component ${dependencies})

target_link_libraries(vertex_detector_component
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${TENSORRT_LIBRARY}
  ${TENSORRT_ONNX_PARSER}
  cudart
)

# Link TBB if available
if(TBB_FOUND)
  target_link_libraries(vertex_detector_component TBB::tbb)
endif()

# Rejestracja jako komponent
rclcpp_components_register_nodes(vertex_detector_component 
  "vertex_detector::VertexDetectorNode")

# ============================================================================
# STANDALONE EXECUTABLE
# ============================================================================

add_executable(vertex_detector_standalone
  src/vertex_detector_node.cpp
  src/image_processor.cpp
  src/nms_filter.cpp
)

ament_target_dependencies(vertex_detector_standalone ${dependencies})

target_link_libraries(vertex_detector_standalone
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${TENSORRT_LIBRARY}
  ${TENSORRT_ONNX_PARSER}
  cudart
)

# Link TBB if available
if(TBB_FOUND)
  target_link_libraries(vertex_detector_standalone TBB::tbb)
endif()

# ============================================================================
# INSTALL
# ============================================================================

install(TARGETS
  vertex_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  vertex_detector_standalone
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts (if they exist)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(PROGRAMS
    scripts/stereo_viewer.py
    scripts/stereo_viewer2.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

# ============================================================================
# EXPORTS
# ============================================================================

ament_export_include_directories(include)
ament_export_libraries(vertex_detector_component)
ament_export_dependencies(${dependencies})

ament_package()