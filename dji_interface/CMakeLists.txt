cmake_minimum_required(VERSION 2.8.3)
project(dji_interface)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME}_lib src/dji_interface.cpp)

cs_add_executable(${PROJECT_NAME}_node src/dji_interface_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

cs_install()
cs_export()
