if(SISL_FOUND)
    list(APPEND OPTIONAL_TESTS
        test_Spline.cpp)
endif()

rock_testsuite(test_base_types test.cpp
    test_samples_Sonar.cpp
    test_Eigen.cpp
    test_Timeout.cpp
    ${OPTIONAL_TESTS}
    DEPS base-types
    DEPS_PKGCONFIG base-logging)

rock_executable(benchmark benchmark.cpp bench_func.cpp
    DEPS base-types
    DEPS_PKGCONFIG base-logging
    NOINSTALL)

