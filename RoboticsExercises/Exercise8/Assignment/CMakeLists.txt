# Test CMake version
cmake_minimum_required(VERSION 3.10)

# The name of the project
project(ConstrainedPathplanning)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(RobWork REQUIRED)

add_executable(ConstrainedPathplanning main.cpp)
target_link_libraries(ConstrainedPathplanning ${ROBWORK_LIBRARIES})
