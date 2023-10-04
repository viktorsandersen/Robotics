# Test CMake version
cmake_minimum_required(VERSION 3.10)

# The name of the project
project("Hello RobWork")
message(${PROJECT_NAME} ":")

# Use c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set build type to release
set(CMAKE_BUILD_TYPE Release)
message("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
find_package(RobWork REQUIRED)

# Add the source files to build
set(SrcFiles src/HelloRobWork.cpp)

# The shared library to build:
add_executable(HelloRobWork ${SrcFiles})
target_link_libraries(HelloRobWork ${ROBWORK_LIBRARIES})

message(${PROJECT_NAME} " done!")
