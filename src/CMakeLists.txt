set(SOURCES
        Angle.cpp
        Pose.cpp
        TransformWithCovariance.cpp
        TwistWithCovariance.cpp
        Waypoint.cpp
)

set(HEADERS 
        Logging.hpp 
        Singleton.hpp 
        logging/logging_iostream_style.h 
        logging/logging_printf_style.h
        Wrench.hpp
)


# Using SISL as optional dependency
find_package(SISL)
if(SISL_FOUND)
    rock_library(base ${SOURCES} Spline.cpp
	    DEPS_CMAKE SISL
	    HEADERS ${HEADERS} Spline.hpp
	    DEPS_PKGCONFIG base-logging)
    add_definitions(-DSISL_FOUND)
else(SISL_FOUND)
    message(STATUS "Compiling ${PROJECT_NAME} without 'spline' support")
    rock_library(base ${SOURCES}
	    HEADERS ${HEADERS}
	    DEPS_PKGCONFIG base-logging)
endif(SISL_FOUND)
    
configure_file(${CMAKE_SOURCE_DIR}/base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
install(FILES ${CMAKE_SOURCE_DIR}/src/Spline.hpp
	DESTINATION include/base/geometry)

