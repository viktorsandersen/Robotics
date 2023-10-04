cmake_minimum_required(VERSION 3.10.0)

project(CubicSpline)
find_package(RobWork REQUIRED)
find_package(Qt6 COMPONENTS Core Widgets Charts QUIET)

add_executable(ex5_cubicSpline main.cpp)
target_link_libraries(ex5_cubicSpline ${ROBWORK_LIBRARIES} ${Qt6Core_LIBRARIES} ${Qt6Widgets_LIBRARIES} ${Qt6Charts_LIBRARIES})
