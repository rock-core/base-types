# Using SISL as optional dependency
find_package(SISL)
if(SISL_FOUND)
    rock_library(base spline.cpp logging.cpp
	    DEPS_CMAKE SISL
	    HEADERS logging.h singleton.h)
else(SISL_FOUND)
    message(STATUS "Compiling ${PROJECT_NAME} without 'spline' support")
    rock_library(base logging.cpp
	    HEADERS logging.h singleton.h)
endif(SISL_FOUND)
    
configure_file(${CMAKE_SOURCE_DIR}/base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
install(FILES ${CMAKE_SOURCE_DIR}/src/spline.h
	DESTINATION include/base/geometry)

