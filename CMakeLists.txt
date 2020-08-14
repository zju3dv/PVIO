cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(pvio-superbuild)

include(CMakeDependentOption)

option(PVIO_DEBUG "..." ON)
option(PVIO_ENABLE_FORENSICS "..." ON)
option(PVIO_ENABLE_THREADING "..." OFF)
option(PVIO_ENABLE_PLANE_CONSTRAINT "..." ON)

add_subdirectory(depends)
add_subdirectory(pvio)
add_subdirectory(pvio-extra)
add_subdirectory(pvio-pc)
