# This flag is used by people that don't want to install boost but still use
# base/types. It downloads boost and installs the header-only parts of it, which
# is really the only thing base/types needs
set(NO_BOOST_DEPENDENCY false
    CACHE BOOL "set to true to force cmake to download and install the header-only parts of boost. BOOST_RELEASE_URL and BOOST_SUBDIRECTORY allow to tune what should actually be downloaded")
set(BOOST_RELEASE_URL "http://sourceforge.net/projects/boost/files/boost/1.55.0/boost_1_55_0.tar.bz2/download"
    CACHE STRING "the URL from which boost should be downloaded if NO_BOOST_DEPENDENCY is set. The value of BOOST_SUBDIRECTORY has to be set accordingly")
set(BOOST_SUBDIRECTORY "boost_1_55_0"
    CACHE STRING "the first directory in the boost release tarball pointed by BOOST_RELEASE_URL (i.e. the first level directory that appears when unpacking the tarball). Used only if NO_BOOST_DEPENDENCY is set")
set(BOOST_LOCAL_TARBALL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/boost.tar.bz2"
    CACHE STRING "the place where the downloaded boost release is saved if NO_BOOST_DEPENDENCY is set")
mark_as_advanced(NO_BOOST_DEPENDENCY BOOST_RELEASE_URL BOOST_SUBDIRECTORY BOOST_LOCAL_TARBALL_PATH)

macro(DOWNLOAD_BOOST_IF_NEEDED)
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/external/${BOOST_SUBDIRECTORY})
        if (NOT EXISTS ${BOOST_LOCAL_TARBALL_PATH})
            message(STATUS "downloading boost release tarball from ${BOOST_RELEASE_URL}")
            file(DOWNLOAD "${BOOST_RELEASE_URL}" ${BOOST_LOCAL_TARBALL_PATH}.partial
                STATUS download_status SHOW_PROGRESS)
            if (NOT download_status)
                message(FATAL_ERROR "could not download the boost tarball from ${BOOST_LOCAL_TARBALL_PATH} and NO_BOOST_DEPENDENCY was set")
            endif()
            file(RENAME ${BOOST_LOCAL_TARBALL_PATH}.partial ${BOOST_LOCAL_TARBALL_PATH})
        endif()
        message(STATUS "extracting boost release tarball downloaded in ${BOOST_LOCAL_TARBALL_PATH} into ${CMAKE_CURRENT_SOURCE_DIR}/external")
        execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external
            COMMAND ${CMAKE_COMMAND} -E tar xzf ${BOOST_LOCAL_TARBALL_PATH})
    else()
        message(STATUS "NO_BOOST_DEPENDENCY is set, but the locally downloaded version seem up to date, not re-downloading")
    endif()
endmacro()
