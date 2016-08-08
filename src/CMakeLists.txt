rock_library(
    base-types 
        Angle.cpp
        JointLimitRange.cpp
        Pose.cpp
        Pressure.cpp
        Spline.cpp
        Temperature.cpp
        Time.cpp
        TimeMark.cpp
        Timeout.cpp
        Trajectory.cpp
        TransformWithCovariance.cpp
        TwistWithCovariance.cpp
        Waypoint.cpp
    HEADERS
        Angle.hpp
        CircularBuffer.hpp
        Deprecated.hpp
        Eigen.hpp
        Float.hpp
        JointLimitRange.cpp
        Logging.hpp
        Point.hpp
        Pose.hpp
        Pressure.hpp
        Singleton.hpp 
        Spline.hpp
        Temperature.hpp
        Time.hpp
        TimeMark.hpp
        Timeout.hpp
        Trajectory.hpp
        TransformWithCovariance.hpp
        TwistWithCovariance.hpp
        Waypoint.hpp
        Wrench.hpp
        logging/logging_iostream_style.h 
        logging/logging_printf_style.h
    DEPS_CMAKE 
        SISL
    DEPS_PKGCONFIG 
        base-logging
        eigen3
)

install(FILES ${CMAKE_SOURCE_DIR}/src/Spline.hpp
	DESTINATION include/base/geometry)

configure_file(base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
