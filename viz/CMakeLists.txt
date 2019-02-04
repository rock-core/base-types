rock_vizkit_plugin(base-viz
    PluginLoader.cpp Uncertainty.cpp Vizkit3DHelper.cpp
    MOC 
        DistanceImageVisualization.cpp 
        LaserScanVisualization.cpp 
        MotionCommandVisualization.cpp 
        RigidBodyStateVisualization.cpp 
        BodyStateVisualization.cpp 
        TrajectoryVisualization.cpp 
        WaypointVisualization.cpp 
        SonarGroundDistanceVisualization.cpp 
        SonarBeamVisualization.cpp
        SonarVisualization.cpp
        PointcloudVisualization.cpp
        DepthMapVisualization.cpp
    HEADERS 
        Uncertainty.hpp 
        Vizkit3DHelper.hpp 
        DistanceImageVisualization.hpp
        LaserScanVisualization.hpp 
        MotionCommandVisualization.hpp 
        RigidBodyStateVisualization.hpp 
        BodyStateVisualization.hpp 
        TrajectoryVisualization.hpp 
        WaypointVisualization.hpp 
        SonarGroundDistanceVisualization.hpp 
        SonarBeamVisualization.hpp
        SonarVisualization.hpp
        PointcloudVisualization.hpp
        DepthMapVisualization.hpp
    DEPS base-types
    LIBS ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG base-logging
)
