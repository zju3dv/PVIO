cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(pvio-extra)

set(PVIO_EXTRA_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PVIO_EXTRA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(pvio-extra-common INTERFACE IMPORTED GLOBAL)
target_include_directories(pvio-extra-common
    INTERFACE
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
        $<BUILD_INTERFACE:${PVIO_EXTRA_INCLUDE_DIR}>
)
target_link_libraries(pvio-extra-common
    INTERFACE
        pvio
)

if(TARGET opencv-dep)
    add_library(pvio-extra-opencv-image
        ${PVIO_EXTRA_INCLUDE_DIR}/pvio/extra/opencv_image.h
        ${PVIO_EXTRA_SOURCE_DIR}/pvio/extra/opencv_image.cpp
        ${PVIO_EXTRA_INCLUDE_DIR}/pvio/extra/image_undistorter.h
    )
    target_link_libraries(pvio-extra-opencv-image
        PUBLIC
            pvio-extra-common
            opencv-dep
    )
endif()

if(TARGET yaml-cpp-dep)
    add_library(pvio-extra-yaml-config
        ${PVIO_EXTRA_INCLUDE_DIR}/pvio/extra/yaml_config.h
        ${PVIO_EXTRA_SOURCE_DIR}/pvio/extra/yaml_config.cpp
    )
    target_link_libraries(pvio-extra-yaml-config
        PUBLIC
            pvio-extra-common
            yaml-cpp-dep
    )
endif()

set(PVIO_EXTRA_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
