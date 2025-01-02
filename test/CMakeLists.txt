if(SISL_FOUND)
    list(APPEND OPTIONAL_TESTS
        test_Spline.cpp)
endif()

rock_testsuite(test_base_types test.cpp
    test_Angle.cpp
    test_Eigen.cpp
    test_JointLimits.cpp
    test_RigidBodyStateSE3.cpp
    test_Temperature.cpp
    test_Time.cpp
    test_Timeout.cpp
    test_samples_Sonar.cpp
    ${OPTIONAL_TESTS}
    DEPS base-types
    DEPS_PKGCONFIG base-logging)

rock_executable(benchmark benchmark.cpp bench_func.cpp
    DEPS base-types
    DEPS_PKGCONFIG base-logging
    NOINSTALL)
rock_executable(test_DepthMapVisualization test_DepthMapVisualization.cpp
    DEPS base-types base-viz
    DEPS_PKGCONFIG vizkit3d
    LIBS ${Boost_SYSTEM_LIBRARY}
    NOINSTALL)

