cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(pvio-pc VERSION 0.2.0 LANGUAGES CXX)

if(NOT PVIO_ENABLE_FORENSICS)
message(WARNING "pvio-pc can compile With PVIO_ENABLE_FORENSICS turned OFF, but it will show no visualizations at all.")
endif()

if(TARGET pvio-extra-opencv-image AND TARGET pvio-extra-yaml-config AND TARGET sensors-toolkit-dep AND TARGET nanovis-dep)

add_executable(pvio-pc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dataset_reader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dataset_reader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/euroc_dataset_reader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/euroc_dataset_reader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tum_dataset_reader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tum_dataset_reader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/legacy_sensors_dataset_reader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/legacy_sensors_dataset_reader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/output_writer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors_dataset_reader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors_dataset_reader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

target_include_directories(pvio-pc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(pvio-pc
    PRIVATE
        pvio-extra-opencv-image
        pvio-extra-yaml-config
        sensors-toolkit-dep
        nanovis-dep
)

endif()
