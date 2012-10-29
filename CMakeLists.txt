cmake_minimum_required(VERSION 2.6)
set(ENV{ROCK_CMAKE_MACROS} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)
rock_init(base 1.0)

rock_export_includedir(${PROJECT_SOURCE_DIR}/src base/geometry)
include_directories(BEFORE ${PROJECT_SOURCE_DIR})

rock_find_cmake(Boost REQUIRED)
rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

rock_standard_layout()

rock_testsuite(test test.cpp DEPS base)

configure_file(base-types.pc.in base-types.pc @ONLY)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/base DESTINATION include)
install(FILES ${CMAKE_BINARY_DIR}/base-types.pc DESTINATION lib/pkgconfig)

configure_file(cmake/rock-config.cmake.in rock-config.cmake @ONLY)
install(FILES
    cmake/FindBoost.cmake
    cmake/FindGem.cmake
    cmake/FindNumpy.cmake
    cmake/FindRuby.cmake
    cmake/FindSISL.cmake
    cmake/ListMacros.cmake
    cmake/PythonBind.cmake
    cmake/PythonInstallPath.cmake
    cmake/Rock.cmake
    cmake/RockRuby.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/rock-config.cmake
    DESTINATION share/rock/cmake)

