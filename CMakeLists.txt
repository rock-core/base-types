PROJECT(base)
cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    add_definitions (-Wall)
endif (CXX_SUPPORTS_WALL)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

add_subdirectory(src)

add_executable(test test.cpp)
target_link_libraries(test)

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")

CONFIGURE_FILE(base-types.pc.in base-types.pc @ONLY)
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/base DESTINATION include)
INSTALL(FILES ${CMAKE_BINARY_DIR}/base-types.pc DESTINATION lib/pkgconfig)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)

