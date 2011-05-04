rock_library(base spline.cpp logging.cpp
	DEPS_CMAKE SISL
	HEADERS logging.h singleton.h)
    
configure_file(${CMAKE_SOURCE_DIR}/base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
install(FILES ${CMAKE_SOURCE_DIR}/src/spline.h
	DESTINATION include/base/geometry)

