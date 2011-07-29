set(SOURCES 
        logging/logging_printf_style.cpp)

set(HEADERS logging.h 
        logging/logging_printf_style.h
        logging/logging_iostream_style.h
        singleton.h)

# Using SISL as optional dependency
find_package(SISL)
if(SISL_FOUND)
    rock_library(base ${SOURCES} spline.cpp
	    DEPS_CMAKE SISL
	    HEADERS ${HEADERS})
else(SISL_FOUND)
    message(STATUS "Compiling ${PROJECT_NAME} without 'spline' support")
    rock_library(base ${SOURCES}
	    HEADERS ${HEADERS})
endif(SISL_FOUND)
    
configure_file(${CMAKE_SOURCE_DIR}/base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
install(FILES ${CMAKE_SOURCE_DIR}/src/spline.h
	DESTINATION include/base/geometry)

