include_directories(${CMAKE_BINARY_DIR}/include)

find_package(SISL)
if(SISL_FOUND)
    add_definitions(-DSISL_FOUND)
    rock_add_ruby_package(base
        EXT_RICE base_types_ruby Eigen.cpp Spline.cpp init.cpp)
else(SISL_FOUND)
    rock_add_ruby_package(base
        EXT_RICE base_types_ruby Eigen.cpp init.cpp)
endif(SISL_FOUND)

if (base_AVAILABLE)
    if(SISL_FOUND)
        install(DIRECTORY lib/sisl
            DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})
    endif(SISL_FOUND)
    install(FILES lib/eigen.rb
        DESTINATION ${RUBY_LIBRARY_INSTALL_DIR})
        target_link_libraries(base_types_ruby base-types)
        rock_typelib_ruby_plugin(lib/base/typelib_plugin.rb RENAME base_types.rb)
endif()

