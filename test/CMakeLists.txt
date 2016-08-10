rock_testsuite(test_base_types test.cpp
    test_samples_Sonar.cpp
    test_Eigen.cpp
    DEPS base
    DEPS_PKGCONFIG base-logging)
rock_executable(benchmark benchmark.cpp bench_func.cpp
    DEPS base
    DEPS_PKGCONFIG base-logging
    NOINSTALL)
