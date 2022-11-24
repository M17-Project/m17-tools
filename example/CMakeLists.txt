cmake_minimum_required(VERSION 3.9)

project(m17-example
    VERSION 0.1
    DESCRIPTION "Standalone example M17 project"
    LANGUAGES CXX)

find_package(m17cxx CONFIG REQUIRED)
if(${m17cxx_FOUND})
    message(STATUS "Found m17cxx")
    message(STATUS "  M17 Library version: " ${m17cxx_VERSION})
    message(STATUS "  M17 Library location: " ${m17cxx_DIR})
else(${m17cxx_FOUND})
    message(FATAL_ERROR "Could not locate m17cxx")
endif()

add_executable(example example.cpp)
target_link_libraries(example m17cxx)
