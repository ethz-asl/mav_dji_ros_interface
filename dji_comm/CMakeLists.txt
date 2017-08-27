cmake_minimum_required(VERSION 2.8.3)
project(dji_comm)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++11)

cs_add_library(dji_serial_port_lib src/dji_serial_port.cpp)

cs_add_library(${PROJECT_NAME}_lib src/dji_comm.cpp)
target_link_libraries(${PROJECT_NAME}_lib dji_serial_port_lib)


cs_install()
cs_export()
