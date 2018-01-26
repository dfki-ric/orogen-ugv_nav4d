PROJECT(ugv_nav4d)
cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
INCLUDE(ugv_nav4dBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)


add_subdirectory(init)