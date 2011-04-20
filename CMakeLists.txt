PROJECT(base)
cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Symlink to allow correct include statement
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/include/)
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME})
include_directories(${PROJECT_BINARY_DIR}/include/)

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    add_definitions (-Wall)
endif (CXX_SUPPORTS_WALL)

add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
#add_definitions(-DBASE_LONG_NAMES)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

add_subdirectory(src)

add_executable(test test.cpp)
target_link_libraries(test ${PROJECT_NAME})

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")

CONFIGURE_FILE(base-types.pc.in base-types.pc @ONLY)
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/base DESTINATION include)
INSTALL(FILES ${CMAKE_BINARY_DIR}/base-types.pc DESTINATION lib/pkgconfig)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)

