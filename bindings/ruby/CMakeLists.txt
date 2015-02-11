include_directories(${CMAKE_BINARY_DIR}/include)

find_package(SISL)
if(SISL_FOUND)
  add_definition(SISL_FOUND)
  rock_add_ruby_package(base
        EXT_RICE base_types_ruby Eigen.cpp Spline.cpp init.cpp)
    if (base_AVAILABLE)
        install(DIRECTORY lib/sisl
            DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})
        install(FILES lib/eigen.rb
            DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})
        target_link_libraries(base_types_ruby base)
        rock_typelib_ruby_plugin(lib/base/typelib_plugin.rb RENAME base_types.rb)
    endif()
else(SISL_FOUND)
    rock_add_ruby_package(base
        EXT_RICE base_types_ruby Eigen.cpp init.cpp)
    if (base_AVAILABLE)
        install(FILES lib/eigen.rb
            DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})
        target_link_libraries(base_types_ruby base)
        rock_typelib_ruby_plugin(lib/base/typelib_plugin.rb RENAME base_types.rb)
    endif()
endif(SISL_FOUND)

