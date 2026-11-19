rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

include(RockQt)

rock_qt_vizkit_plugin(
    TARGETPREFIX base-viz
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES
        PluginLoader.cpp Uncertainty.cpp Vizkit3DHelper.cpp
        DistanceImageVisualization.cpp
        LaserScanVisualization.cpp
        MotionCommandVisualization.cpp
        RigidBodyStateVisualization.cpp
        BodyStateVisualization.cpp
        WaypointVisualization.cpp
        SonarGroundDistanceVisualization.cpp
        SonarBeamVisualization.cpp
        SonarVisualization.cpp
        PointcloudVisualization.cpp
        DepthMapVisualization.cpp
        OrientedBoundingBoxVisualization.cpp
        TrajectoryVisualization.cpp
    MOC
        PluginLoader.hpp
        DistanceImageVisualization.hpp
        LaserScanVisualization.hpp
        MotionCommandVisualization.hpp
        RigidBodyStateVisualization.hpp
        BodyStateVisualization.hpp
        WaypointVisualization.hpp
        SonarGroundDistanceVisualization.hpp
        SonarBeamVisualization.hpp
        SonarVisualization.hpp
        PointcloudVisualization.hpp
        DepthMapVisualization.hpp
        OrientedBoundingBoxVisualization.hpp
        TrajectoryVisualization.hpp
    HEADERS
        Uncertainty.hpp
        Vizkit3DHelper.hpp
        DistanceImageVisualization.hpp
        LaserScanVisualization.hpp
        MotionCommandVisualization.hpp
        RigidBodyStateVisualization.hpp
        BodyStateVisualization.hpp
        WaypointVisualization.hpp
        SonarGroundDistanceVisualization.hpp
        SonarBeamVisualization.hpp
        SonarVisualization.hpp
        PointcloudVisualization.hpp
        DepthMapVisualization.hpp
        OrientedBoundingBoxVisualization.hpp
        TrajectoryVisualization.hpp
    DEPS base-types
    LIBS ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG base-logging PrimitivesFactory
)
