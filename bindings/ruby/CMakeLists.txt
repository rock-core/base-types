include_directories(${CMAKE_BINARY_DIR}/include)
rock_add_ruby_package(base
    EXT_RICE base_types_ruby Eigen.cpp Spline.cpp init.cpp)
if (base_AVAILABLE)
    install(FILES lib/eigen.rb
        DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})
    target_link_libraries(base_types_ruby base)
    rock_typelib_ruby_plugin(lib/base/typelib_plugin.rb RENAME base_types.rb)
endif()

