include_directories(BEFORE ${PROJECT_SOURCE_DIR})
find_package(SISL)
if (SISL_FOUND)
    include_directories(${SISL_INCLUDE_DIRS})
    install(FILES ${CMAKE_SOURCE_DIR}/src/spline.h
        DESTINATION include/base/geometry)
    list(APPEND LIBBASE_FILES ${CMAKE_SOURCE_DIR}/src/spline.cpp)
    list(APPEND LIBBASE_DEPENDENCIES ${SISL_LIBRARIES})
endif()

list(APPEND LIBBASE_FILES ${CMAKE_SOURCE_DIR}/src/logging.cpp)
install(FILES ${CMAKE_SOURCE_DIR}/src/logging.h
              ${CMAKE_SOURCE_DIR}/src/singleton.h
        DESTINATION include/base)

if (LIBBASE_FILES)
    add_library(base SHARED ${LIBBASE_FILES})
    target_link_libraries(base ${LIBBASE_DEPENDENCIES})
    install(TARGETS base
        LIBRARY DESTINATION lib)
    configure_file(${CMAKE_SOURCE_DIR}/base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
endif()

