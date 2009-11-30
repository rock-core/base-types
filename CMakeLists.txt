PROJECT(base)
cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

add_executable(test test.cpp)
target_link_libraries(test)

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")

CONFIGURE_FILE(base-types.pc.in base-types.pc @ONLY)
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/base DESTINATION include)
INSTALL(FILES ${CMAKE_BINARY_DIR}/base-types.pc DESTINATION lib/pkgconfig)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

