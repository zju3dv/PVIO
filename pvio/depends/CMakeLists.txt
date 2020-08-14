cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(pvio-lib-dependencies)

include(FetchContent)

# Eigen3
# if(PVIO_BUILD_EIGEN)
#     FetchContent_Declare(
#         eigen
#         GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
#         GIT_TAG        3.3.7
#     )
#     FetchContent_GetProperties(eigen)
#     if(NOT eigen_POPULATED)
#         message(STATUS "Fetching eigen sources")
#         FetchContent_Populate(eigen)
#         message(STATUS "Fetching eigen sources - done")
#     endif()
#     add_library(eigen-dep INTERFACE IMPORTED GLOBAL)
#     target_include_directories(eigen-dep INTERFACE ${eigen_SOURCE_DIR})
#     set(EIGEN_DIR ${eigen_SOURCE_DIR} CACHE PATH "..." FORCE)
#     set(EIGEN_INCLUDE_DIR ${eigen_SOURCE_DIR} CACHE PATH "..." FORCE)
# else()
    find_package(Eigen3 REQUIRED)
    add_library(eigen-dep INTERFACE IMPORTED GLOBAL)
    target_link_libraries(eigen-dep INTERFACE Eigen3::Eigen)
# endif()

# ceres-solver
if(PVIO_BUILD_CERES)
    FetchContent_Declare(
        ceres-solver
        GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
        GIT_TAG        1.14.0
    )
    FetchContent_GetProperties(ceres-solver)
    if(NOT ceres-solver_POPULATED)
        message(STATUS "Fetching ceres-solver sources")
        FetchContent_Populate(ceres-solver)
        message(STATUS "Fetching ceres-solver sources - done")
        message(STATUS "Configuring ceres-solver")
        set(ceres-solver_CONFIGURATIONS
            -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
            -DCMAKE_BUILD_TYPE=Release
            -DBUILD_EXAMPLES=OFF
            -DBUILD_TESTING=OFF
            -DMINIGLOG=ON
            -DGFLAGS=OFF
            -DCXX11=ON
            -DCXX11_THREADS=ON
            -DTBB=OFF
            -DOPENMP=OFF
            -DLAPACK=ON
            -DMINIGLOG_MAX_LOG_LEVEL=0
        )
        execute_process(
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_STRIP_TRAILING_WHITESPACE
            COMMAND ${CMAKE_COMMAND} -S${ceres-solver_SOURCE_DIR} -B${ceres-solver_BINARY_DIR} ${ceres-solver_CONFIGURATIONS}
        )
        message(STATUS "Configuring ceres-solver - done")
    endif()
    add_custom_target(ceres-solver-build
        COMMAND ${CMAKE_COMMAND} --build ${ceres-solver_BINARY_DIR} --parallel 6
    )
    add_library(ceres-dep INTERFACE IMPORTED GLOBAL)
    add_dependencies(ceres-dep ceres-solver-build)
    target_include_directories(ceres-dep
        INTERFACE
            ${ceres-solver_BINARY_DIR}/config
            ${ceres-solver_SOURCE_DIR}/internal/ceres/miniglog
            ${ceres-solver_SOURCE_DIR}/include
    )
    target_link_libraries(ceres-dep
        INTERFACE
            ${ceres-solver_BINARY_DIR}/lib/libceres.a
    )
    set(CERES_LIBRARY_DIR ${ceres-solver_BINARY_DIR} CACHE INTERNAL "")
else()
    find_package(Ceres REQUIRED)
    add_library(ceres-dep INTERFACE IMPORTED GLOBAL)
    target_include_directories(ceres-dep
        INTERFACE
            ${CERES_INCLUDE_DIRS}
    )
    target_link_libraries(ceres-dep
        INTERFACE
            ${CERES_LIBRARIES}
    )
endif()
