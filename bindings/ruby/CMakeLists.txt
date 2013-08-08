include(RockRuby)
include_directories(${CMAKE_BINARY_DIR}/include)

set(EXT_SOURCES
    ext/base_types/Eigen.cpp
    ext/base_types/Spline.cpp
    ext/base_types/init.cpp)

rock_ruby_rice_extension(base_types_ruby ${EXT_SOURCES})
target_link_libraries(base_types_ruby ${PROJECT_NAME})

set(RUBY_FILES
    lib/base/float.rb
    lib/base/typelib_plugin.rb
    lib/base/geometry
    lib/base/typelib
    )

rock_ruby_library(base ${RUBY_FILES})

install(FILES lib/eigen.rb DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})

