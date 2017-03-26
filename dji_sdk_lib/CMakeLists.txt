cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_lib)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-fPIC)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++11)

## Declare a C++ library
FILE(GLOB DJI_SDK_LIB_SOURCES src/*.cpp)

cs_add_library(dji_sdk_lib
	STATIC
	${DJI_SDK_LIB_SOURCES}
)

cs_install()
cs_export()
