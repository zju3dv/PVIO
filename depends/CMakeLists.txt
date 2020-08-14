cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(pvio-common-dependencies)

include(FetchContent)

# opencv
if(NOT PVIO_DISABLE_OPENCV)
    find_package(OpenCV REQUIRED)
    add_library(opencv-dep INTERFACE IMPORTED GLOBAL)
    target_include_directories(opencv-dep INTERFACE ${OpenCV_INCLUDE_DIR})
    target_link_libraries(opencv-dep INTERFACE ${OpenCV_LIBS})
endif()

# yaml-cpp
if(NOT PVIO_DISABLE_YAMLCPP)
find_package(yaml-cpp REQUIRED)
add_library(yaml-cpp-dep INTERFACE IMPORTED GLOBAL)
target_link_libraries(yaml-cpp-dep INTERFACE yaml-cpp)
endif()

# nanovis
if(NOT PVIO_DISABLE_NANOVIS)
FetchContent_Declare(
    nanovis
    GIT_REPOSITORY https://github.com/itsuhane/NanoVis.git
    GIT_TAG        91076d5
)
FetchContent_GetProperties(nanovis)
if(NOT nanovis_POPULATED)
    message(STATUS "Fetching NanoVis sources")
    FetchContent_Populate(nanovis)
    message(STATUS "Fetching NanoVis sources - done")
endif()
add_subdirectory(${nanovis_SOURCE_DIR} ${nanovis_BINARY_DIR})
add_library(nanovis-dep INTERFACE IMPORTED GLOBAL)
target_link_libraries(nanovis-dep INTERFACE nanovis)
endif()

# sensors-toolkit
if(NOT PVIO_DISABLE_SENSORS_TOOLKIT)
FetchContent_Declare(
    sensors-toolkit
    GIT_REPOSITORY https://github.com/itsuhane/sensors-toolkit.git
    GIT_TAG        a912f85
)
FetchContent_GetProperties(sensors-toolkit)
if(NOT sensors-toolkit_POPULATED)
    message(STATUS "Fetching sensors-toolkit sources")
    FetchContent_Populate(sensors-toolkit)
    message(STATUS "Fetching sensors-toolkit sources - done")
endif()
set(SENSORS_BUILD_LIB ON CACHE BOOL INTERNAL)
set(SENSORS_BUILD_PC OFF CACHE BOOL INTERNAL)
add_subdirectory(${sensors-toolkit_SOURCE_DIR} ${sensors-toolkit_BINARY_DIR})
add_library(sensors-toolkit-dep INTERFACE IMPORTED GLOBAL)
target_link_libraries(sensors-toolkit-dep INTERFACE sensors)
endif()
