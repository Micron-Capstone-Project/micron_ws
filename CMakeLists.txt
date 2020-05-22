cmake_minimum_required(VERSION 3.0.0)
project(micron)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# FIX THIS
find_package(Boost REQUIRED COMPONENTS system thread python)
find_package(PythonLibs REQUIRED)

set(jacl_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../jacl-1.0.0/share/jacl/cmake)
find_package(jacl REQUIRED)

add_subdirectory(include/serial_wrapper)
add_subdirectory(include/mqtt_handler)

add_library(micron include/micron/micron.hpp)
set_target_properties(micron PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(micron
    PRIVATE
    ${JACL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    include
)
target_link_libraries(micron PRIVATE jacl_lib mqtt_handler serial_wrapper)

add_executable(micron_go micron_go.cpp)
target_include_directories(micron_go PRIVATE
    include
    ${JACL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)
target_link_libraries(micron_go PRIVATE micron)
