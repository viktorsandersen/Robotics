# Test CMake version
cmake_minimum_required(VERSION 3.10)

# The name of the project
project(Pathplanning)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(RobWork REQUIRED)

add_executable(Pathplanning main.cpp)
target_link_libraries(Pathplanning ${ROBWORK_LIBRARIES})
