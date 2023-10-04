# Test CMake version
cmake_minimum_required(VERSION 3.10)

# The name of the project
project(ConstrainedPathplanning)

add_compile_options(-std=c++17)

# We use the settings that robwork studio uses
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE relwithdebinfo)
endif()

find_package(RobWork REQUIRED)

add_executable(ex8_ConstrainedPathplanning main.cpp)
target_link_libraries(ex8_ConstrainedPathplanning ${ROBWORK_LIBRARIES})
