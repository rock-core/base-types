
find_package(SISL)

set(sisl_hpp "")
set(sisl_cpp "")

#compile without Spline and Trajectory if SISL is not found
if(SISL_FOUND)
  set(sisl_hpp Spline.hpp
               Trajectory.hpp)
  set(sisl_cpp Spline.cpp
               Trajectory.cpp)
  include_directories(${SISL_INCLUDE_DIRS})
else()
  message(WARNING "SISL not found. Compiling without spline and trajectory support!")
endif()

rock_library(
    base-types
        Angle.cpp
        JointLimitRange.cpp
        JointLimits.cpp
        JointState.cpp
        JointsTrajectory.cpp
        JointTransform.cpp
        Pose.cpp
        Pressure.cpp
        ${sisl_cpp}
        Temperature.cpp
        Time.cpp
        TimeMark.cpp
        Timeout.cpp
        TransformWithCovariance.cpp
        TwistWithCovariance.cpp
        Waypoint.cpp
        commands/Motion2D.cpp
        samples/BodyState.cpp
        samples/DepthMap.cpp
        samples/DistanceImage.cpp
        samples/Frame.cpp
        samples/Joints.cpp
        samples/LaserScan.cpp
        samples/Pressure.cpp
        samples/RigidBodyAcceleration.cpp
        samples/RigidBodyState.cpp
        samples/Sonar.cpp
        samples/SonarBeam.cpp
        samples/SonarScan.cpp
        samples/PoseWithCovariance.cpp
    HEADERS
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
        Singleton.hpp
        Temperature.hpp
        Time.hpp
        ${sisl_hpp}
        TimeMark.hpp
        Timeout.hpp
        TransformWithCovariance.hpp
        TwistWithCovariance.hpp
        Waypoint.hpp
        Wrench.hpp
        commands/Joints.hpp
        commands/Motion2D.hpp
        commands/Speed6D.hpp
	commands/LinearAngular6DCommand.hpp
        logging/logging_iostream_style.h
        logging/logging_printf_style.h
        samples/BodyState.hpp
        samples/CommandSamples.hpp
        samples/DepthMap.hpp
        samples/DistanceImage.hpp
        samples/Frame.hpp
        samples/IMUSensors.hpp
        samples/Joints.hpp
        samples/LaserScan.hpp
        samples/Pointcloud.hpp
        samples/Pressure.hpp
        samples/RigidBodyAcceleration.hpp
        samples/RigidBodyState.hpp
        samples/Sonar.hpp
        samples/SonarBeam.hpp
        samples/SonarScan.hpp
        samples/PoseWithCovariance.hpp
        samples/Wrench.hpp
        samples/Wrenches.hpp
        templates/TimeStamped.hpp
    DEPS_PKGCONFIG
        base-logging
        eigen3
)

if(SISL_FOUND)
  target_link_libraries(base-types ${SISL_LIBRARIES})
  
  install(FILES ${CMAKE_SOURCE_DIR}/src/Spline.hpp
  	DESTINATION include/base/geometry)
endif()

configure_file(base-lib.pc.in ${CMAKE_BINARY_DIR}/base-lib.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/base-lib.pc DESTINATION lib/pkgconfig)
