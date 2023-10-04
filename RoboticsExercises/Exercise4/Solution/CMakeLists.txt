cmake_minimum_required(VERSION 3.10.0)

project(LinearInterpolator)

add_compile_options(-std=c++17)

# We use the settings that robwork studio uses
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE relwithdebinfo)
endif()

find_package(RobWork REQUIRED)

add_executable(ex4_linInterpolator Main-4.cpp)
target_link_libraries(ex4_linInterpolator ${ROBWORK_LIBRARIES})

add_executable(ex4_linInterpolatorBonus Main-bonus.cpp)
target_link_libraries(ex4_linInterpolatorBonus ${ROBWORK_LIBRARIES})
