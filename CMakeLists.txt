PROJECT(ugv_nav4d)
cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DENABLE_DEBUG_DRAWINGS)

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
INCLUDE(ugv_nav4dBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)


#disabled due to circular dependency with lib_init_optional
#add_subdirectory(init)