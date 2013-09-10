set(SOURCES 
        logging/logging_printf_style.cpp)

set(HEADERS Logging.hpp
        logging/logging_printf_style.h
        logging/logging_iostream_style.h
        Singleton.hpp)

# Using SISL as optional dependency
find_package(SISL)
if(SISL_FOUND)
    rock_library(base ${SOURCES} Spline.cpp
	    DEPS_CMAKE SISL
	    HEADERS ${HEADERS})
    add_definitions(-DSISL_FOUND)
else(SISL_FOUND)
    message(STATUS "Compiling ${PROJECT_NAME} without 'spline' support")
    rock_library(base ${SOURCES}
	    HEADERS ${HEADERS})
endif(SISL_FOUND)
    
configure_file(${CMAKE_SOURCE_DIR}/base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
install(FILES ${CMAKE_SOURCE_DIR}/src/Spline.hpp
	DESTINATION include/base/geometry)

