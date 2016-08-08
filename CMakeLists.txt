cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(base 1.0)

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
    rock_find_cmake(Boost COMPONENTS system REQUIRED)
endif()

rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

# For forward compatibility. Test was a toplevel target
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/test)
    if (NOT IS_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test)
        FILE(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/test)
    endif()
endif()

rock_standard_layout()

