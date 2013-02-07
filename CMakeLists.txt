cmake_minimum_required(VERSION 2.6)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Rock.cmake)
rock_init(base 1.0)

rock_export_includedir(${PROJECT_SOURCE_DIR}/src base/geometry)
include_directories(BEFORE ${PROJECT_SOURCE_DIR})

rock_find_cmake(Boost REQUIRED)
rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

# For forward compatibility. Test was a toplevel target
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/test)
    if (NOT IS_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test)
        FILE(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/test)
    endif()
endif()

rock_standard_layout()

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

