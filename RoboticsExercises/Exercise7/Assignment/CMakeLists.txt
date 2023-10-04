# Test CMake version
cmake_minimum_required(VERSION 3.10)

# The name of the project
project(PathPruning)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(RobWork REQUIRED)

add_executable(PathPruning main.cpp)
target_link_libraries(PathPruning ${ROBWORK_LIBRARIES})
