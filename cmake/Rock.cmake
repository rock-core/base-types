macro(rock_use_full_rpath install_rpath)
    # use, i.e. don't skip the full RPATH for the build tree
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

    # when building, don't use the install RPATH already
    # (but later on when installing)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

    # the RPATH to be used when installing
    SET(CMAKE_INSTALL_RPATH ${install_rpath})

    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the insgall RPATH
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endmacro()

## Main initialization for Rock CMake projects
macro (rock_init PROJECT_NAME PROJECT_VERSION)
    project(${PROJECT_NAME})
    set(PROJECT_VERSION ${PROJECT_VERSION})
    list(APPEND CMAKE_MODULE_PATH $ENV{ROCK_CMAKE_MACROS})
    rock_use_full_rpath("${CMAKE_INSTALL_PREFIX}/lib")


    include(CheckCXXCompilerFlag)
    include(FindPkgConfig)
    CHECK_CXX_COMPILER_FLAG(-Wall ROCK_CXX_SUPPORTS_WALL)
    if (ROCK_CXX_SUPPORTS_WALL)
        add_definitions(-Wall)
    endif()
    add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
endmacro()

function(rock_export_includedir DIR TARGET_DIR)
    execute_process(
        COMMAND cmake -E make_directory ${PROJECT_BINARY_DIR}/include)
    execute_process(
        COMMAND cmake -E create_symlink ${DIR} ${PROJECT_BINARY_DIR}/include/${TARGET_DIR})
    include_directories(BEFORE ${PROJECT_BINARY_DIR}/include)
endfunction()

function(rock_add_source_dir DIR TARGET_DIR)
    rock_export_includedir(${CMAKE_CURRENT_SOURCE_DIR}/${DIR}
        ${TARGET_DIR})
    add_subdirectory(${DIR})
endfunction()

macro(rock_standard_layout)
    if (EXISTS ${PROJECT_SOURCE_DIR}/Doxyfile.in)
        find_package(Doxygen)
        if (DOXYGEN_FOUND)
            if (DOXYGEN_DOT_EXECUTABLE)
                SET(DOXYGEN_DOT_FOUND YES)
            elSE(DOXYGEN_DOT_EXECUTABLE)
                SET(DOXYGEN_DOT_FOUND NO)
                SET(DOXYGEN_DOT_EXECUTABLE "")
            endif(DOXYGEN_DOT_EXECUTABLE)
            configure_file(Doxyfile.in Doxyfile @ONLY)
            add_custom_target(doc doxygen Doxyfile)
        endif(DOXYGEN_FOUND)
    endif()

    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/src)
        rock_add_source_dir(src ${PROJECT_NAME})
    endif()

    # Test for known types of Rock subprojects
    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/viz)
        option(ROCK_VIZ_ENABLED "set to OFF to disable the visualization plugin. Visualization plugins are automatically disabled if Rock's vizkit is not available" ON)
        if (ROCK_VIZ_ENABLED)
            rock_find_pkgconfig(vizkit vizkit)
            if (vizkit_FOUND)
                message(STATUS "vizkit found ... building the vizkit plugin")
                add_subdirectory(viz)
            else()
                message(STATUS "vizkit not found ... NOT building the vizkit plugin")
            endif()
        else()
            message(STATUS "visualization plugins disabled as ROCK_VIZ_ENABLED is set to OFF")
        endif()
    endif()

    if (IS_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
        option(ROCK_TEST_ENABLED "set to OFF to disable the unit tests. Tests are automatically disabled if the boost unit test framework is not available" ON)
        if (ROCK_TEST_ENABLED)
            find_package(Boost REQUIRED COMPONENTS unit_test_framework)
            if (Boost_UNIT_TEST_FRAMEWORK_FOUND)
                message(STATUS "boost/test found ... building test the suite")
                add_subdirectory(test)
            else()
                message(STATUS "boost/test not found ... NOT building the test suite")
            endif()
        else()
            message(STATUS "unit tests disabled as ROCK_TEST_ENABLED is set to OFF")
        endif()
    endif()

    if (IS_DIRECTORY ${PROJECT_SOURCE_DIR}/ruby)
        if (EXISTS ${PROJECT_SOURCE_DIR}/ruby/CMakeLists.txt)
            include(RockRuby)
            if (RUBY_EXTENSIONS_AVAILABLE)
                add_subdirectory(ruby)
            endif()
        endif()
    endif()

    if (IS_DIRECTORY ${PROJECT_SOURCE_DIR}/configuration)
	install(DIRECTORY ${PROJECT_SOURCE_DIR}/configuration/ DESTINATION configuration/${PROJECT_NAME}
	        FILES_MATCHING PATTERN "*" 
	                       PATTERN "*.pc" EXCLUDE)
    endif()

endmacro()

## Like pkg_check_modules, but calls include_directories and link_directories
# using the resulting information
macro (rock_find_pkgconfig VARIABLE)
    if (NOT ${VARIABLE}_FOUND)
        pkg_check_modules(${VARIABLE} ${ARGN})
        foreach(${VARIABLE}_lib ${${VARIABLE}_LIBRARIES})
          set(_${VARIABLE}_lib NOTFOUND)
          find_library(_${VARIABLE}_lib NAMES ${${VARIABLE}_lib} HINTS ${${VARIABLE}_LIBRARY_DIRS})
          if (NOT _${VARIABLE}_lib)
            set(_${VARIABLE}_lib ${${VARIABLE}_lib})
          endif()
          list(APPEND _${VARIABLE}_LIBRARIES ${_${VARIABLE}_lib})
        endforeach()
        list(APPEND _${VARIABLE}_LIBRARIES ${${VARIABLE}_LDFLAGS_OTHER})
        set(${VARIABLE}_LIBRARIES ${_${VARIABLE}_LIBRARIES} CACHE INTERNAL "")
    endif()

    add_definitions(${${VARIABLE}_CFLAGS})
    include_directories(${${VARIABLE}_INCLUDE_DIRS})
endmacro()

## Like find_package, but calls include_directories and link_directories using
# the resulting information
macro (rock_find_cmake VARIABLE)
    find_package(${VARIABLE} ${ARGN})
    string(TOUPPER ${VARIABLE} UPPER_VARIABLE)
    add_definitions(${${VARIABLE}_CFLAGS})
    add_definitions(${${UPPER_VARIABLE}_CFLAGS})
    include_directories(${${VARIABLE}_INCLUDE_DIRS})
    include_directories(${${VARIABLE}_INCLUDE_DIR})
    link_directories(${${VARIABLE}_LIBRARY_DIRS})
    link_directories(${${VARIABLE}_LIBRARY_DIR})
    include_directories(${${UPPER_VARIABLE}_INCLUDE_DIRS})
    include_directories(${${UPPER_VARIABLE}_INCLUDE_DIR})
    link_directories(${${UPPER_VARIABLE}_LIBRARY_DIRS})
    link_directories(${${UPPER_VARIABLE}_LIBRARY_DIR})
endmacro()

macro (rock_find_qt4) 
    find_package(Qt4 REQUIRED QtCore QtGui)
    add_definitions(${QT_DEFINITIONS})
    include_directories(${QT_INCLUDE_DIR})
    link_directories(${QT_LIBRARY_DIR})
endmacro()

## Common parsing of parameters for all the C/C++ target types
macro(rock_target_definition TARGET_NAME)
    set(${TARGET_NAME}_INSTALL ON)
    set(ROCK_TARGET_AVAILABLE_MODES "SOURCES;HEADERS;DEPS;DEPS_PKGCONFIG;DEPS_CMAKE;MOC;LIBS")

    set(${TARGET_NAME}_MODE "SOURCES")
    foreach(ELEMENT ${ARGN})
        list(FIND ROCK_TARGET_AVAILABLE_MODES "${ELEMENT}" IS_KNOWN_MODE)
        if ("${ELEMENT}" STREQUAL "LIBS")
            set(${TARGET_NAME}_MODE DEPENDENT_LIBS)
        elseif (IS_KNOWN_MODE GREATER -1)
            set(${TARGET_NAME}_MODE "${ELEMENT}")
        elseif("${ELEMENT}" STREQUAL "NOINSTALL")
            set(${TARGET_NAME}_INSTALL OFF)
        else()
            list(APPEND ${TARGET_NAME}_${${TARGET_NAME}_MODE} "${ELEMENT}")
        endif()
    endforeach()

    foreach (internal_dep ${${TARGET_NAME}_DEPS})
        get_property(internal_dep_DEPS TARGET ${internal_dep}
            PROPERTY DEPS_PKGCONFIG)
        list(APPEND ${TARGET_NAME}_DEPS_PKGCONFIG ${internal_dep_DEPS})
    endforeach()
    
    foreach (pkgconfig_pkg ${${TARGET_NAME}_DEPS_PKGCONFIG})
        rock_find_pkgconfig(${pkgconfig_pkg}_PKGCONFIG REQUIRED ${pkgconfig_pkg})
    endforeach()
    foreach (cmake_pkg ${${TARGET_NAME}_DEPS_CMAKE})
        rock_find_cmake(${cmake_pkg} REQUIRED)
    endforeach()

    list(LENGTH ${TARGET_NAME}_MOC QT_SOURCE_LENGTH)
    if (QT_SOURCE_LENGTH GREATER 0)
        rock_find_qt4()
        list(APPEND ${TARGET_NAME}_DEPENDENT_LIBS ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}) 
        QT4_WRAP_CPP(${TARGET_NAME}_MOC_SRCS ${${TARGET_NAME}_MOC})
        list(APPEND ${TARGET_NAME}_SOURCES ${${TARGET_NAME}_MOC_SRCS})
    endif()
endmacro()

## Common post-target-definition setup for all C/C++ targets
macro(rock_target_setup TARGET_NAME)
    set_property(TARGET ${TARGET_NAME}
        PROPERTY DEPS_PKGCONFIG ${${TARGET_NAME}_DEPS_PKGCONFIG})

    foreach (pkgconfig_pkg ${${TARGET_NAME}_DEPS_PKGCONFIG})
        target_link_libraries(${TARGET_NAME} ${${pkgconfig_pkg}_PKGCONFIG_LIBRARIES})
    endforeach()
    target_link_libraries(${TARGET_NAME} ${${TARGET_NAME}_DEPS})
    target_link_libraries(${TARGET_NAME} ${${TARGET_NAME}_DEPENDENT_LIBS})
    foreach (cmake_pkg ${${TARGET_NAME}_DEPS_CMAKE})
        string(TOUPPER ${cmake_pkg} UPPER_cmake_pkg)
        target_link_libraries(${TARGET_NAME} ${${cmake_pkg}_LIBRARIES} ${${cmake_pkg}_LIBRARY})
        target_link_libraries(${TARGET_NAME} ${${UPPER_cmake_pkg}_LIBRARIES} ${${UPPER_cmake_pkg}_LIBRARY})
    endforeach()
endmacro()

## Defines a new C++ executable
#
# rock_executable(name
#     SOURCES source.cpp source1.cpp ...
#     [DEPS target1 target2 target3]
#     [DEPS_PKGCONFIG pkg1 pkg2 pkg3]
#     [DEPS_CMAKE pkg1 pkg2 pkg3]
#     [MOC qtsource1.hpp qtsource2.hpp])
#
# Creates a C++ executable and (optionally) installs it
#
# The following arguments are mandatory:
#
# SOURCES: list of the C++ sources that should be built into that library
#
# The following optional arguments are available:
#
# DEPS: lists the other targets from this CMake project against which the
# library should be linked
# DEPS_PKGCONFIG: list of pkg-config packages that the library depends upon. The
# necessary link and compilation flags are added
# DEPS_CMAKE: list of packages which can be found with CMake's find_package,
# that the library depends upon. It is assumed that the Find*.cmake scripts
# follow the cmake accepted standard for variable naming
# MOC: if the library is Qt-based, a list of headers that should be processed by
# moc. The resulting implementation files are built into the library
function(rock_executable TARGET_NAME)
    rock_target_definition(${TARGET_NAME} ${ARGN})

    add_executable(${TARGET_NAME} ${${TARGET_NAME}_SOURCES})
    rock_target_setup(${TARGET_NAME})

    if (${TARGET_NAME}_INSTALL)
        install(TARGETS ${TARGET_NAME}
            RUNTIME DESTINATION bin)
    endif()
endfunction()

## Common setup for libraries in Rock. Used by rock_library and
# rock_vizkit_plugin
macro(rock_library_common TARGET_NAME)
    rock_target_definition(${TARGET_NAME} ${ARGN})

    add_library(${TARGET_NAME} SHARED ${${TARGET_NAME}_SOURCES})
    rock_target_setup(${TARGET_NAME})

    foreach(pkgname ${${TARGET_NAME}_DEPS_PKGCONFIG})
        set(DEPS_PKGCONFIG "${DEPS_PKGCONFIG} ${pkgname}")
    endforeach()

    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in
            ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
        if (${TARGET_NAME}_INSTALL)
            install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc
                DESTINATION lib/pkgconfig)
        endif()
    endif()
endmacro()

## Defines a new shared library
#
# rock_library(name
#     SOURCES source.cpp source1.cpp ...
#     [DEPS target1 target2 target3]
#     [DEPS_PKGCONFIG pkg1 pkg2 pkg3]
#     [DEPS_CMAKE pkg1 pkg2 pkg3]
#     [HEADERS header1.hpp header2.hpp header3.hpp ...]
#     [MOC qtsource1.hpp qtsource2.hpp]
#     [NOINSTALL])
#
# Creates and (optionally) installs a shared library.
#
# As with all rock libraries, the target must have a pkg-config file along, that
# gets generated and (optionally) installed by the macro. The pkg-config file
# needs to be in the same directory and called <name>.pc.in
# 
# The following arguments are mandatory:
#
# SOURCES: list of the C++ sources that should be built into that library
#
# The following optional arguments are available:
#
# DEPS: lists the other targets from this CMake project against which the
# library should be linked
# DEPS_PKGCONFIG: list of pkg-config packages that the library depends upon. The
# necessary link and compilation flags are added
# DEPS_CMAKE: list of packages which can be found with CMake's find_package,
# that the library depends upon. It is assumed that the Find*.cmake scripts
# follow the cmake accepted standard for variable naming
# HEADERS: a list of headers that should be installed with the library. They get
# installed in include/project_name
# MOC: if the library is Qt-based, a list of headers that should be processed by
# moc. The resulting implementation files are built into the library
# NOINSTALL: by default, the library gets installed on 'make install'. If this
# argument is given, this is turned off
function(rock_library TARGET_NAME)
    rock_library_common(${TARGET_NAME} ${ARGN})

    if (${TARGET_NAME}_INSTALL)
        install(TARGETS ${TARGET_NAME}
            LIBRARY DESTINATION lib
            # On Windows the dll part of a library is treated as RUNTIME target
            # and the corresponding import library is treated as ARCHIVE target
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin)
        # Install headers and keep directory structure
        foreach(HEADER ${${TARGET_NAME}_HEADERS})
            string(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
            install(FILES ${HEADER} DESTINATION include/${PROJECT_NAME}/${DIR})
        endforeach(HEADER)
    endif()
endfunction()

## Defines a new vizkit plugin
#
# rock_vizkit_plugin(name
#     SOURCES source.cpp source1.cpp ...
#     [DEPS target1 target2 target3]
#     [DEPS_PKGCONFIG pkg1 pkg2 pkg3]
#     [DEPS_CMAKE pkg1 pkg2 pkg3]
#     [HEADERS header1.hpp header2.hpp header3.hpp ...]
#     [MOC qtsource1.hpp qtsource2.hpp]
#     [NOINSTALL])
#
# Creates and (optionally) installs a shared library that defines a vizkit
# plugin. In Rock, vizkit is the base for data display. Vizkit plugins are
# plugins to the 3D display in vizkit.
#
# The library gets linked against the vizkit libraries automatically (no
# need to list them in DEPS_PKGCONFIG). Moreoer, unlike with a normal shared
# library, the headers get installed in include/vizkit
# 
# The following arguments are mandatory:
#
# SOURCES: list of the C++ sources that should be built into that library
#
# The following optional arguments are available:
#
# DEPS: lists the other targets from this CMake project against which the
# library should be linked
# DEPS_PKGCONFIG: list of pkg-config packages that the library depends upon. The
# necessary link and compilation flags are added
# DEPS_CMAKE: list of packages which can be found with CMake's find_package,
# that the library depends upon. It is assumed that the Find*.cmake scripts
# follow the cmake accepted standard for variable naming
# HEADERS: a list of headers that should be installed with the library. They get
# installed in include/project_name
# MOC: if the library is Qt-based, a list of headers that should be processed by
# moc. The resulting implementation files are built into the library
# NOINSTALL: by default, the library gets installed on 'make install'. If this
# argument is given, this is turned off
function(rock_vizkit_plugin TARGET_NAME)
    rock_library_common(${TARGET_NAME} ${ARGN} DEPS_PKGCONFIG vizkit)
    if (${TARGET_NAME}_INSTALL)
        install(TARGETS ${TARGET_NAME}
            LIBRARY DESTINATION lib)
        install(FILES ${${TARGET_NAME}_HEADERS}
            DESTINATION include/vizkit)
        install(FILES vizkit_plugin.rb
            DESTINATION lib/qt/designer/widgets
            RENAME ${PROJECT_NAME}_vizkit.rb
            OPTIONAL)
    endif()
endfunction()

## Defines a new vizkit widget
#
# rock_vizkit_widget(name
#     SOURCES source.cpp source1.cpp ...
#     [DEPS target1 target2 target3]
#     [DEPS_PKGCONFIG pkg1 pkg2 pkg3]
#     [DEPS_CMAKE pkg1 pkg2 pkg3]
#     [HEADERS header1.hpp header2.hpp header3.hpp ...]
#     [MOC qtsource1.hpp qtsource2.hpp]
#     [NOINSTALL])
#
# Creates and (optionally) installs a shared library that defines a vizkit
# widget. In Rock, vizkit is the base for data display. Vizkit widgets are
# Qt designer widgets that can be seamlessly integrated in the vizkit framework.
#
# If a file exists that goes by the name <name>.rb exists, it is assumed to be
# a ruby extension used to extend the C++ interface in ruby scripting. It gets
# installed in share/vizkit/cplusplus_extensions, where vizkit is looking for
# it.
# 
# The library gets linked against the QtCore librariy automatically (no
# need to list them in DEPS_PKGCONFIG). Moreover, unlike with a normal shared
# library, the headers get installed in include/package_name
# 
# The following arguments are mandatory:
#
# SOURCES: list of the C++ sources that should be built into that library
# MOC:     a list of headers that should be processed by moc. The resulting 
#          implementation files are built into the library
#
# The following optional arguments are available:
#
# DEPS: lists the other targets from this CMake project against which the
# library should be linked
# DEPS_PKGCONFIG: list of pkg-config packages that the library depends upon. The
# necessary link and compilation flags are added
# DEPS_CMAKE: list of packages which can be found with CMake's find_package,
# that the library depends upon. It is assumed that the Find*.cmake scripts
# follow the cmake accepted standard for variable naming
# HEADERS: a list of headers that should be installed with the library. They get
# installed in include/project_name
# NOINSTALL: by default, the library gets installed on 'make install'. If this
# argument is given, this is turned off
function(rock_vizkit_widget TARGET_NAME)
    rock_library_common(${TARGET_NAME} ${ARGN})
    if (${TARGET_NAME}_INSTALL)
        install(TARGETS ${TARGET_NAME}
            LIBRARY DESTINATION lib/qt/designer)
        install(FILES ${${TARGET_NAME}_HEADERS}
            DESTINATION include/${PROJECT_NAME})
        install(FILES ${TARGET_NAME}.rb
            DESTINATION share/vizkit/ext
            OPTIONAL)
        install(FILES vizkit_widget.rb
            DESTINATION lib/qt/designer/cplusplus_extensions
            RENAME ${PROJECT_NAME}_vizkit.rb
            OPTIONAL)
    endif()
endfunction()

## Defines a new C++ test suite
#
# rock_testsuite(name
#     SOURCES source.cpp source1.cpp ...
#     [DEPS target1 target2 target3]
#     [DEPS_PKGCONFIG pkg1 pkg2 pkg3]
#     [DEPS_CMAKE pkg1 pkg2 pkg3]
#     [MOC qtsource1.hpp qtsource2.hpp])
#
# Creates a C++ test suite that is using the boost unit test framework
#
# The following arguments are mandatory:
#
# SOURCES: list of the C++ sources that should be built into that library
#
# The following optional arguments are available:
#
# DEPS: lists the other targets from this CMake project against which the
# library should be linked
# DEPS_PKGCONFIG: list of pkg-config packages that the library depends upon. The
# necessary link and compilation flags are added
# DEPS_CMAKE: list of packages which can be found with CMake's find_package,
# that the library depends upon. It is assumed that the Find*.cmake scripts
# follow the cmake accepted standard for variable naming
# MOC: if the library is Qt-based, a list of headers that should be processed by
# moc. The resulting implementation files are built into the library
function(rock_testsuite TARGET_NAME)
    rock_executable(${TARGET_NAME} ${ARGN}
        NOINSTALL)
    target_link_libraries(${TARGET_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
    add_test(RockTestSuite ${EXECUTABLE_OUTPUT_PATH}/${TARGET_NAME})
endfunction()

