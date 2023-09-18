if(SISL_FOUND)
    message(STATUS "SISL found: ${SISL_LIBRARIES}")
    list(APPEND OPTIONAL_HPP
        Spline.hpp
        Trajectory.hpp
    )
    list(APPEND OPTIONAL_CPP
        Spline.cpp
        Trajectory.cpp
    )
endif()

rock_library(
    base-types 
        Acceleration.cpp
        Angle.cpp
        JointLimitRange.cpp
        JointLimits.cpp
        JointState.cpp
        JointsTrajectory.cpp
        JointTransform.cpp
        Pose.cpp
        Pressure.cpp
        RigidBodyStateSE3.cpp
        Temperature.cpp
        Time.cpp
        TimeMark.cpp
        Timeout.cpp
        TransformWithCovariance.cpp
        Twist.cpp
        TwistWithCovariance.cpp
        Waypoint.cpp
        Wrench.cpp
        commands/Motion2D.cpp
        samples/BodyState.cpp
        samples/BoundingBox.cpp
        samples/DepthMap.cpp
        samples/DistanceImage.cpp
        samples/Frame.cpp
        samples/Joints.cpp
        samples/LaserScan.cpp
        samples/OrientedBoundingBox.cpp
        samples/PoseWithCovariance.cpp
        samples/Pressure.cpp
        samples/RigidBodyAcceleration.cpp
        samples/RigidBodyState.cpp
        samples/Sonar.cpp
        samples/SonarBeam.cpp
        samples/SonarScan.cpp
        samples/Temperature.cpp
        ${OPTIONAL_CPP}
    HEADERS
        Acceleration.hpp
        Angle.hpp
        CircularBuffer.hpp
        Deprecated.hpp
        Eigen.hpp
        Float.hpp
        JointLimitRange.hpp
        JointLimits.hpp
        JointState.hpp
        JointsTrajectory.hpp
        JointTransform.hpp
        Logging.hpp
        Matrix.hpp
        NamedVector.hpp
        Point.hpp
        Pose.hpp
        Pressure.hpp
        RigidBodyStateSE3.hpp
        Singleton.hpp 
        Spline.hpp
        Temperature.hpp
        Time.hpp
        TimeMark.hpp
        Timeout.hpp
        TransformWithCovariance.hpp
        Twist.hpp
        TwistWithCovariance.hpp
        Waypoint.hpp
        Wrench.hpp
        commands/Joints.hpp
        commands/Motion2D.hpp
        commands/Speed6D.hpp
        commands/LinearAngular6DCommand.hpp
        logging/logging_iostream_style.h 
        logging/logging_printf_style.h
        samples/Acceleration.hpp
        samples/BoundingBox.hpp
        samples/BodyState.hpp
        samples/CommandSamples.hpp
        samples/DepthMap.hpp
        samples/DistanceImage.hpp
        samples/Frame.hpp
        samples/IMUSensors.hpp
        samples/Joints.hpp
        samples/LaserScan.hpp
        samples/OrientedBoundingBox.hpp
        samples/Pointcloud.hpp
        samples/Pressure.hpp
        samples/RigidBodyAcceleration.hpp
        samples/RigidBodyState.hpp
        samples/RigidBodyStateSE3.hpp
        samples/RigidBodyStateSE3Vector.hpp
        samples/Sonar.hpp
        samples/SonarBeam.hpp
        samples/SonarScan.hpp
        samples/PoseWithCovariance.hpp
        samples/Twist.hpp
        samples/Temperature.hpp
        templates/TimeStamped.hpp
        samples/Wrench.hpp
        samples/Wrenches.hpp
        samples/Event.hpp
        samples/EventArray.hpp
        ${OPTIONAL_HPP}
    DEPS_CMAKE JPEG
    DEPS_PKGCONFIG 
        base-logging
        eigen3
)

if(SISL_FOUND)
    target_link_libraries(base-types ${SISL_LIBRARIES})
    install(FILES ${CMAKE_SOURCE_DIR}/src/Spline.hpp
            DESTINATION include/base/geometry)
    configure_file(base-types-sisl.pc.in ${CMAKE_BINARY_DIR}/base-types-sisl.pc @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/base-types-sisl.pc DESTINATION lib/pkgconfig)
endif()

configure_file(base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
