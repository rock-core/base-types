# This module finds the Ruby package and defines a ADD_RUBY_EXTENSION macro to
# build and install Ruby extensions
# 
# Upon loading, it sets a RUBY_EXTENSIONS_AVAILABLE variable to true if Ruby
# extensions can be built.
#
# The ADD_RUBY_EXTENSION macro can be used as follows:
#  ADD_RUBY_EXTENSION(target_name source1 source2 source3 ...)
#
# 

FIND_PACKAGE(Ruby)
if (NOT RUBY_FOUND)
    MESSAGE(STATUS "Ruby library not found. Skipping Ruby parts for this package")
else()
    MESSAGE(STATUS "Ruby library found")
    function(ROCK_RUBY_LIBRARY libname)
        if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${libname}.rb)
            install(FILES ${libname}.rb
                DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})
            list(REMOVE_ITEM ARGN ${libname}.rb)
        endif()
        if (IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${libname})
            install(DIRECTORY ${libname}
                DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})
            list(REMOVE_ITEM ARGN ${libname})
        endif()

        foreach(to_install ${ARGN})
            if (IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${to_install})
                install(DIRECTORY ${to_install}
                    DESTINATION ${RUBY_LIBRARY_INSTALL_DIR}/${libname})
            else()
                install(FILES ${to_install}
                    DESTINATION ${RUBY_LIBRARY_INSTALL_DIR}/${libname})
            endif()
        endforeach()
    endfunction()

    function(ROCK_LOG_MIGRATION)
        if (EXISTS ${CMAKE_SOURCE_DIR}/src/log_migration.rb)
            configure_file(${CMAKE_SOURCE_DIR}/src/log_migration.rb
                ${CMAKE_BINARY_DIR}/log_migration-${PROJECT_NAME}.rb COPYONLY)
            install(FILES ${CMAKE_BINARY_DIR}/log_migration-${PROJECT_NAME}.rb
                    DESTINATION share/rock/log/migration)
        endif()
    endfunction()
endif()

IF(NOT RUBY_INCLUDE_PATH)
    MESSAGE(STATUS "Ruby library not found. Cannot build Ruby extensions")
    SET(RUBY_EXTENSIONS_AVAILABLE FALSE)
ELSEIF(NOT RUBY_EXTENSIONS_AVAILABLE)
    SET(RUBY_EXTENSIONS_AVAILABLE TRUE)
    STRING(REGEX REPLACE ".*lib(32|64)?/?" "lib/" RUBY_EXTENSIONS_INSTALL_DIR ${RUBY_ARCH_DIR})
    STRING(REGEX REPLACE ".*lib(32|64)?/?" "lib/" RUBY_LIBRARY_INSTALL_DIR ${RUBY_RUBY_LIB_PATH})

    FIND_PROGRAM(RDOC_EXECUTABLE NAMES rdoc1.9 rdoc1.8 rdoc)

    EXECUTE_PROCESS(COMMAND ${RUBY_EXECUTABLE} -r rbconfig -e "puts RUBY_VERSION"
       OUTPUT_VARIABLE RUBY_VERSION)
    STRING(REPLACE "\n" "" RUBY_VERSION ${RUBY_VERSION})
    STRING(REGEX MATCH "^1\\.9" RUBY_19 ${RUBY_VERSION})
    STRING(REGEX MATCH "^1\\.9\\.1" RUBY_191 ${RUBY_VERSION})
    message(STATUS "found Ruby version ${RUBY_VERSION}")

    EXECUTE_PROCESS(COMMAND ${RUBY_EXECUTABLE} -r rbconfig -e "puts RbConfig::CONFIG['CFLAGS']"
       OUTPUT_VARIABLE RUBY_CFLAGS)
    STRING(REPLACE "\n" "" RUBY_CFLAGS ${RUBY_CFLAGS})

    function(ROCK_TYPELIB_RUBY_PLUGIN)
        install(FILES ${ARGN}
            DESTINATION share/typelib/ruby)
    endfunction()

    function(ROCK_LOG_EXPORT)
        if (EXISTS ${CMAKE_SOURCE_DIR}/src/log_export.rb)
            configure_file(${CMAKE_SOURCE_DIR}/src/log_export.rb
                ${CMAKE_BINARY_DIR}/log_export-${PROJECT_NAME}.rb COPYONLY)
            install(FILES ${CMAKE_BINARY_DIR}/log_export-${PROJECT_NAME}.rb
                    DESTINATION share/rock/log/export)
        endif()
    endfunction()

    function(ROCK_RUBY_EXTENSION target)
	INCLUDE_DIRECTORIES(${RUBY_INCLUDE_PATH})
        list(GET ${RUBY_INCLUDE_PATH} 0 rubylib_path)
	GET_FILENAME_COMPONENT(rubylib_path ${rubylib_path} PATH)
	LINK_DIRECTORIES(${rubylib_path})

        if (RUBY_191)
            add_definitions(-DRUBY_191)
        elseif (RUBY_19)
            add_definitions(-DRUBY_19)
        endif()

	SET_SOURCE_FILES_PROPERTIES(${ARGN} PROPERTIES COMPILE_FLAGS "${RUBY_CFLAGS}")
        rock_library_common(${target} MODULE ${ARGN})
        target_link_libraries(${target} ${RUBY_LIBRARY})
        set_target_properties(${target} PROPERTIES
            LINK_FLAGS "-z noexecstack")
	SET_TARGET_PROPERTIES(${target} PROPERTIES PREFIX "")
    endfunction()

    function(ROCK_RUBY_RICE_EXTENSION target)
        find_package(Gem COMPONENTS rice)
        if (GEM_FOUND)
            ROCK_RUBY_EXTENSION(${target} ${ARGN})
	    include_directories(${GEM_INCLUDE_DIRS})
	    target_link_libraries(${target} ${GEM_LIBRARIES})

	    install(TARGETS ${target} LIBRARY DESTINATION ${RUBY_EXTENSIONS_INSTALL_DIR})
        else()
            message(STATUS "cannot find the rice gem")
        endif()

    endfunction()
ENDIF(NOT RUBY_INCLUDE_PATH)

