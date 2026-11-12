cmake_minimum_required(VERSION 3.10)
project(pa_lvio)

## Build Options

if("$ENV{ROS_DISTRO}" STREQUAL "lyrical")
    set(CMAKE_CXX_STANDARD 20)
else()
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_BUILD_TYPE STREQUAL Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
endif()

# Locate workspace-local absl installation if not provided by system
find_package(absl QUIET)
if(NOT absl_FOUND)
    if(EXISTS "${PROJECT_SOURCE_DIR}/../../.deps/absl")
        list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/../../.deps/absl")
    elseif(EXISTS "${PROJECT_SOURCE_DIR}/.deps/absl")
        list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/.deps/absl")
    endif()
endif()

# Prefer workspace-local CUDA-enabled OpenCV installation if present
if(EXISTS "${PROJECT_SOURCE_DIR}/../../.deps/opencv/lib/cmake/opencv4")
    set(PA_LVIO_OPENCV_ROOT "${PROJECT_SOURCE_DIR}/../../.deps/opencv")
elseif(EXISTS "${PROJECT_SOURCE_DIR}/.deps/opencv/lib/cmake/opencv4")
    set(PA_LVIO_OPENCV_ROOT "${PROJECT_SOURCE_DIR}/.deps/opencv")
endif()
if(PA_LVIO_OPENCV_ROOT)
    get_filename_component(PA_LVIO_OPENCV_ROOT "${PA_LVIO_OPENCV_ROOT}" REALPATH)
    set(OpenCV_DIR "${PA_LVIO_OPENCV_ROOT}/lib/cmake/opencv4")
    list(PREPEND CMAKE_PREFIX_PATH "${PA_LVIO_OPENCV_ROOT}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
    set(PA_LVIO_EXTRA_RPATH ":${PA_LVIO_OPENCV_ROOT}/lib:/usr/local/cuda/lib64")
endif()

# Pre-find PCL and resolve VTK::jsoncpp transitive target on Ubuntu 26.04+
find_package(PCL REQUIRED)
if(TARGET VTK::jsoncpp AND NOT TARGET JsonCpp::JsonCpp)
    find_package(jsoncpp CONFIG REQUIRED)
endif()

## pa_lvio

# lvio Library
add_subdirectory(lvio)

set(LVIO_LIBRARIES
    lvio
    fileio
    preintegration
    visual
    lidar
    ikd-Tree
    factors
    common)

set_target_properties(${LVIO_LIBRARIES} PROPERTIES
    BUILD_WITH_INSTALL_RPATH FALSE
    INSTALL_RPATH "$ORIGIN${PA_LVIO_EXTRA_RPATH}"
)

## ROS Interface (OpenVINS dual ROS1 / ROS2 style)
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if (catkin_FOUND AND "$ENV{ROS_VERSION}" STREQUAL "1")
    message(STATUS "ROS *1* version found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND AND "$ENV{ROS_VERSION}" STREQUAL "2")
    message(STATUS "ROS *2* version found, building ROS2.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
elseif (catkin_FOUND)
    message(STATUS "ROS *1* version found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND)
    message(STATUS "ROS *2* version found, building ROS2.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else ()
    message(STATUS "No ROS versions found, building core lvio library only")
endif ()
