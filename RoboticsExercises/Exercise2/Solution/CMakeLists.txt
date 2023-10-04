#
# Test CMake version
#
cmake_minimum_required(VERSION 3.10.0)

# The name of the project.
project(kinematics)

add_compile_options(-std=c++17)

# We use the settings that robwork studio uses
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE relwithdebinfo)
endif()

find_package(RobWork REQUIRED)

add_executable(ex2_jacobian src/jacobian.cpp)
target_link_libraries(ex2_jacobian ${ROBWORK_LIBRARIES})

add_executable(ex2_forwardkinematics src/forwardkinematics.cpp)
target_link_libraries(ex2_forwardkinematics ${ROBWORK_LIBRARIES})
