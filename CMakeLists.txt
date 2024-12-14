PROJECT(ugv_nav4d)
cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ENABLE_DEBUG_DRAWINGS TRUE)

if(ENABLE_DEBUG_DRAWINGS)
    message(STATUS "ENABLE_DEBUG_DRAWINGS is defined with value: ${ENABLE_DEBUG_DRAWINGS}")
    add_definitions(-DENABLE_DEBUG_DRAWINGS)
    add_definitions(-DUSE_PORTS)
else()
    message(STATUS "ENABLE_DEBUG_DRAWINGS is set to OFF. Skipped!")
endif()

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
INCLUDE(ugv_nav4dBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)


#disabled due to circular dependency with lib_init_optional
#add_subdirectory(init)
