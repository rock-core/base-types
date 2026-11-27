cmake_minimum_required(VERSION 3.10)
project(base VERSION 1.0 DESCRIPTION "Common types for Rock systems")

find_package(Rock)
rock_init()
rock_feature(NOCURDIR)

# rock_export_includedir(${PROJECT_SOURCE_DIR}/base ${PROJECT_NAME})
rock_export_includedir(${PROJECT_SOURCE_DIR}/src base/geometry)
include_directories(BEFORE ${PROJECT_SOURCE_DIR})

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/DownloadBoost.cmake)

if (INSTALL_BOOST_IF_REQUIRED)
    rock_find_cmake(Boost)

    if (NOT Boost_FOUND)
        # download and install boost header only
        download_boost_if_needed()
        include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/${BOOST_SUBDIRECTORY})
        install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external/${BOOST_SUBDIRECTORY}/boost
            DESTINATION include)
    endif()

else()
    rock_find_cmake(Boost REQUIRED)
endif()

rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

# For forward compatibility. Test was a toplevel target
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/test)
    if (NOT IS_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test)
        FILE(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/test)
    endif()
endif()

# base::geometry::Spline (and base::Trajectory) is implemented either with
# SISL (the default) or with Eigen
option(USE_SISL
    "set to OFF to use the Eigen-based implementation of base::geometry::Spline instead of SISL."
    ON)
if(USE_SISL)
    rock_find_cmake(SISL)
    if(NOT SISL_FOUND)
        message(FATAL_ERROR "The usage of SISL is requested, but the library could not be found."
            " Either make sure SISL is installed, or use the Eigen-based implementation"
            " via -DUSE_SISL=OFF")
    endif(NOT SISL_FOUND)
    message(STATUS "base::geometry::Spline: using the SISL implementation (${SISL_LIBRARIES})")
else()
    message(STATUS "base::geometry::Spline: using the Eigen-based implementation")
endif()
# Whether base::geometry::Spline is implemented with SISL, as a header so that
# it is consistent between base/types and the packages using it
set(BASE_TYPES_SPLINE_USE_SISL ${USE_SISL})
configure_file(src/SplineConfig.hpp.in
    ${CMAKE_BINARY_DIR}/include_generated/base/geometry/SplineConfig.hpp)
include_directories(BEFORE ${CMAKE_BINARY_DIR}/include_generated)
install(FILES ${CMAKE_BINARY_DIR}/include_generated/base/geometry/SplineConfig.hpp
    DESTINATION include/base/geometry)

rock_standard_layout()
