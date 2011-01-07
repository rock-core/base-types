macro(rock_use_full_rpath install_rpath)
    # use, i.e. don't skip the full RPATH for the build tree
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

    # when building, don't use the install RPATH already
    # (but later on when installing)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

    # the RPATH to be used when installing
    SET(CMAKE_INSTALL_RPATH ${install_rpath})

    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endmacro()

## Main initialization for Rock CMake projects
macro (rock_init PROJECT_NAME PROJECT_VERSION)
    project(${PROJECT_NAME})
    set(PROJECT_VERSION ${PROJECT_VERSION})
    rock_use_full_rpath("${CMAKE_INSTALL_PREFIX}/lib")

    include(CheckCXXCompilerFlag)
    include(FindPkgConfig)
    CHECK_CXX_COMPILER_FLAG(-Wall ROCK_CXX_SUPPORTS_WALL)
    if (ROCK_CXX_SUPPORTS_WALL)
        add_definitions(-Wall)
    endif()
endmacro()

function(rock_export_includedir DIR TARGET_DIR)
    execute_process(
        COMMAND cmake -E make_directory ${PROJECT_BINARY_DIR}/include)
    execute_process(
        COMMAND cmake -E create_symlink ${DIR} ${PROJECT_BINARY_DIR}/include/${TARGET_DIR})
    include_directories(BEFORE ${PROJECT_BINARY_DIR}/include)
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

    if (IS_DIRECTORY ${PROJECT_SOURCE_DIR}/src)
        rock_export_includedir(${PROJECT_SOURCE_DIR}/src ${PROJECT_NAME})
        add_subdirectory(src)
    endif()

    # Test for known types of Rock subprojects
    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/viz)
        pkg_check_modules(vizkit vizkit)
        if (vizkit_FOUND)
            message(STATUS "vizkit found ... building the vizkit plugin")
            add_subdirectory(viz)
        else()
            message(STATUS "vizkit not found ... NOT building the vizkit plugin")
        endif()
    endif()

    if (IS_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
        find_package(Boost REQUIRED COMPONENTS unit_test_framework)
        if (Boost_UNIT_TEST_FRAMEWORK_FOUND)
            message(STATUS "boost/test found ... building test the suite")
            add_subdirectory(test)
        else()
            message(STATUS "boost/test not found ... NOT building the test suite")
        endif()
    endif()

    if (IS_DIRECTORY ${PROJECT_SOURCE_DIR}/ruby)
        include(RockRuby)
        if (RUBY_EXTENSIONS_AVAILABLE)
            add_subdirectory(ruby)
        endif()
    endif()
endmacro()

## Like pkg_check_modules, but calls include_directories and link_directories
# using the resulting information
macro (rock_find_pkgconfig VARIABLE)
    pkg_check_modules(${VARIABLE} ${ARGN})
    add_definitions(${${VARIABLE}_CFLAGS})
    include_directories(${${VARIABLE}_INCLUDE_DIRS})
    link_directories(${${VARIABLE}_LIBRARY_DIRS})
endmacro()

## Like find_package, but calls include_directories and link_directories using
# the resulting information
macro (rock_find_cmake VARIABLE)
    find_package(${VARIABLE} ${ARGN})
    include_directories(${${VARIABLE}_INCLUDE_DIRS})
    include_directories(${${VARIABLE}_INCLUDE_DIR})
    link_directories(${${VARIABLE}_LIBRARY_DIRS})
    link_directories(${${VARIABLE}_LIBRARY_DIR})
endmacro()

## Common parsing of parameters for all the C/C++ target types
macro(rock_target_definition TARGET_NAME)
    set(${TARGET_NAME}_INSTALL ON)
    set(ROCK_TARGET_AVAILABLE_MODES "SOURCES;HEADERS;DEPS;DEPS_PKGCONFIG;DEPS_CMAKE;MOC")

    set(${TARGET_NAME}_MODE "SOURCES")
    foreach(ELEMENT ${ARGN})
        list(FIND ROCK_TARGET_AVAILABLE_MODES "${ELEMENT}" IS_KNOWN_MODE)
        if (IS_KNOWN_MODE GREATER -1)
            set(${TARGET_NAME}_MODE "${ELEMENT}")
        elseif("${ELEMENT}" STREQUAL "NOINSTALL")
            set(${TARGET_NAME}_NOINSTALL ON)
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
    foreach (cmake_pkg ${${TARGET_NAME}_DEPS_CMAKE})
        target_link_libraries(${TARGET_NAME} ${${cmake_pkg}_LIBRARIES} ${${cmake_pkg}_LIBRARY})
    endforeach()
endmacro()

## Defines a new executable that follows Rock guidelines
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

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in
        ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
    if (${TARGET_NAME}_INSTALL)
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc
            DESTINATION lib/pkgconfig)
    endif()
endmacro()

## Defines a new shared library
function(rock_library TARGET_NAME)
    rock_library_common(${TARGET_NAME} ${ARGN})

    if (${TARGET_NAME}_INSTALL)
        install(TARGETS ${TARGET_NAME}
            LIBRARY DESTINATION lib)
        install(FILES ${${TARGET_NAME}_HEADERS}
            DESTINATION include/${PROJECT_NAME})
    endif()
endfunction()

## Defines a new vizkit plugin
function(rock_vizkit_plugin TARGET_NAME)
    rock_library_common(${TARGET_NAME} ${ARGN} DEPS_PKGCONFIG vizkit)
    if (${TARGET_NAME}_INSTALL)
        install(TARGETS ${TARGET_NAME}
            LIBRARY DESTINATION lib)
        install(FILES ${${TARGET_NAME}_HEADERS}
            DESTINATION include/vizkit)
    endif()
endfunction()

## Defines a new vizkit plugin
function(rock_vizkit_widget TARGET_NAME)
    rock_export_includedir(${CMAKE_CURRENT_SOURCE_DIR} vizkit)
    rock_library_common(${TARGET_NAME} ${ARGN} DEPS_PKGCONFIG vizkit)
    if (${TARGET_NAME}_INSTALL)
        install(TARGETS ${TARGET_NAME}
            LIBRARY DESTINATION lib)
        install(FILES ${${TARGET_NAME}_HEADERS}
            DESTINATION include/vizkit)
        install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib${TARGET_NAME}.so.rb
            DESTINATION lib OPTIONAL)
    endif()
endfunction()

## Creates a testsuite
function(rock_testsuite TARGET_NAME)
    rock_executable(${TARGET_NAME} ${ARGN}
        NOINSTALL)
    target_link_libraries(${TARGET_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
    add_test(RockTestSuite ${EXECUTABLE_OUTPUT_PATH}/${TARGET_NAME})
endfunction()

