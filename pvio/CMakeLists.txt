cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(pvio-library VERSION 0.3.0 LANGUAGES CXX)

add_subdirectory(depends)

configure_file(cmake/version.h.in include/pvio/version.h)

set(PVIO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PVIO_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PVIO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PVIO_BINARY_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
set(PVIO_TEST_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)

set(PUBLIC_HEADERS
    ${PVIO_INCLUDE_DIR}/pvio/forensics.h
    ${PVIO_INCLUDE_DIR}/pvio/pvio.h
    ${PVIO_BINARY_INCLUDE_DIR}/pvio/version.h
)

set(PUBLIC_SOURCES
)

set(PRIVATE_HEADERS
    ${PVIO_SOURCE_DIR}/pvio/common.h
    ${PVIO_SOURCE_DIR}/pvio/core/core.h
    ${PVIO_SOURCE_DIR}/pvio/core/feature_tracker.h
    ${PVIO_SOURCE_DIR}/pvio/core/frontend_worker.h
    ${PVIO_SOURCE_DIR}/pvio/core/initializer.h
    ${PVIO_SOURCE_DIR}/pvio/core/plane_extractor.h
    ${PVIO_SOURCE_DIR}/pvio/core/sliding_window_tracker.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/cost_function_validator.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/marginalization_error_cost.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/plane_distance_error_cost.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/augmented_plane_distance_error_cost.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/pairwise_augmented_plane_distance_error_cost.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/depth_only_plane_distance_error_cost.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/preintegration_error_cost.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/quaternion_parameterization.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/reprojection_error_cost.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/ceres/solver_options.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/bundle_adjustor.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/factor.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/pnp.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/preintegrator.h
    ${PVIO_SOURCE_DIR}/pvio/estimation/state.h
    ${PVIO_SOURCE_DIR}/pvio/geometry/essential.h
    ${PVIO_SOURCE_DIR}/pvio/geometry/homography.h
    ${PVIO_SOURCE_DIR}/pvio/geometry/lie_algebra.h
    ${PVIO_SOURCE_DIR}/pvio/geometry/stereo.h
    ${PVIO_SOURCE_DIR}/pvio/map/frame.h
    ${PVIO_SOURCE_DIR}/pvio/map/map.h
    ${PVIO_SOURCE_DIR}/pvio/map/plane.h
    ${PVIO_SOURCE_DIR}/pvio/map/track.h
    ${PVIO_SOURCE_DIR}/pvio/utility/debug.h
    ${PVIO_SOURCE_DIR}/pvio/utility/identifiable.h
    ${PVIO_SOURCE_DIR}/pvio/utility/poisson_disk_filter.h
    ${PVIO_SOURCE_DIR}/pvio/utility/sector_area.h
    ${PVIO_SOURCE_DIR}/pvio/utility/random.h
    ${PVIO_SOURCE_DIR}/pvio/utility/ransac.h
    ${PVIO_SOURCE_DIR}/pvio/utility/unique_timer.h
    ${PVIO_SOURCE_DIR}/pvio/utility/worker.h
    ${PVIO_SOURCE_DIR}/pvio/utility/wahba_problem.h
)

set(PRIVATE_SOURCES
    ${PVIO_SOURCE_DIR}/pvio/config.cpp
    ${PVIO_SOURCE_DIR}/pvio/core/core.cpp
    ${PVIO_SOURCE_DIR}/pvio/core/feature_tracker.cpp
    ${PVIO_SOURCE_DIR}/pvio/core/frontend_worker.cpp
    ${PVIO_SOURCE_DIR}/pvio/core/initializer.cpp
    ${PVIO_SOURCE_DIR}/pvio/core/plane_extractor.cpp
    ${PVIO_SOURCE_DIR}/pvio/core/sliding_window_tracker.cpp
    ${PVIO_SOURCE_DIR}/pvio/estimation/bundle_adjustor.cpp
    ${PVIO_SOURCE_DIR}/pvio/estimation/factor.cpp
    ${PVIO_SOURCE_DIR}/pvio/estimation/pnp.cpp
    ${PVIO_SOURCE_DIR}/pvio/estimation/preintegrator.cpp
    ${PVIO_SOURCE_DIR}/pvio/forensics.cpp
    ${PVIO_SOURCE_DIR}/pvio/geometry/essential.cpp
    ${PVIO_SOURCE_DIR}/pvio/geometry/homography.cpp
    ${PVIO_SOURCE_DIR}/pvio/geometry/lie_algebra.cpp
    ${PVIO_SOURCE_DIR}/pvio/geometry/stereo.cpp
    ${PVIO_SOURCE_DIR}/pvio/map/frame.cpp
    ${PVIO_SOURCE_DIR}/pvio/map/map.cpp
    ${PVIO_SOURCE_DIR}/pvio/map/plane.cpp
    ${PVIO_SOURCE_DIR}/pvio/map/track.cpp
    ${PVIO_SOURCE_DIR}/pvio/pvio.cpp
    ${PVIO_SOURCE_DIR}/pvio/utility/debug.cpp
    ${PVIO_SOURCE_DIR}/pvio/utility/worker.cpp
)

add_library(pvio
    ${PUBLIC_HEADERS}
    ${PUBLIC_SOURCES}
    ${PRIVATE_HEADERS}
    ${PRIVATE_SOURCES}
)

target_compile_features(pvio
    PUBLIC
        cxx_std_17
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(pvio
        PRIVATE
            -Qunused-arguments
            -mllvm
            -inline-threshold=5000
    )
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_compile_definitions(pvio
        PRIVATE
            _CRT_SECURE_NO_WARNINGS
    )
endif()

target_include_directories(pvio
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
        $<BUILD_INTERFACE:${PVIO_INCLUDE_DIR}>
        ${PVIO_BINARY_INCLUDE_DIR}
    PRIVATE
        ${PVIO_SOURCE_DIR}
)

target_link_libraries(pvio
    PUBLIC
        eigen-dep
        ceres-dep
)

set(PVIO_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
