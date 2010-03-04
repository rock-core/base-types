find_package(SISL)
if (SISL_FOUND)
    include_directories(${SISL_INCLUDE_DIRS})
    install(FILES ${CMAKE_SOURCE_DIR}/src/spline.h
        DESTINATION include/base/geometry)
    list(APPEND LIBBASE_FILES ${CMAKE_SOURCE_DIR}/src/spline.cpp)
    list(APPEND LIBBASE_DEPENDENCIES ${SISL_LIBRARIES})
endif()

if (LIBBASE_FILES)
    add_library(base SHARED ${LIBBASE_FILES})
    target_link_libraries(base ${LIBBASE_DEPENDENCIES})
    install(TARGETS base
        LIBRARY DESTINATION lib)
    configure_file(${CMAKE_SOURCE_DIR}/base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
endif()

