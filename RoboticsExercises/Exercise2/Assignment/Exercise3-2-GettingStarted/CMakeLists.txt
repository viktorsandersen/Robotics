# Test CMake version
cmake_minimum_required(VERSION 3.10)

# The name of the project
project("Exercise3")
message(${PROJECT_NAME} ":")

# Use c++11
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

# Set build type to release
set(CMAKE_BUILD_TYPE Release)
message("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
find_package(RobWork REQUIRED)

# The shared library to build:
add_executable(exercise32 src/Exercise3-2.cpp)
target_link_libraries(exercise32 ${ROBWORK_LIBRARIES})

