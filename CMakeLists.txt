cmake_minimum_required(VERSION 2.6)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)
rock_init(base 1.0)

rock_export_includedir(${PROJECT_SOURCE_DIR}/src base)
include_directories(BEFORE ${PROJECT_SOURCE_DIR})

rock_find_cmake(Boost REQUIRED)
rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

rock_standard_layout()

rock_testsuite(test test.cpp DEPS base)

configure_file(base-types.pc.in base-types.pc @ONLY)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/base DESTINATION include)
install(FILES ${CMAKE_BINARY_DIR}/base-types.pc DESTINATION lib/pkgconfig)

