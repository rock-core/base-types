rock_vizkit_plugin(base-viz
    PluginLoader.cpp Uncertainty.cpp Vizkit3DHelper.cpp
    MOC 
        DistanceImageVisualization.cpp 
        LaserScanVisualization.cpp 
        MotionCommandVisualization.cpp 
        RigidBodyStateVisualization.cpp 
        TrajectoryVisualization.cpp 
        WaypointVisualization.cpp 
        SonarGroundDistanceVisualization.cpp 
        SonarBeamVisualization.cpp
        PointcloudVisualization.cpp
    HEADERS 
        Uncertainty.hpp 
        Vizkit3DHelper.hpp 
        DistanceImageVisualization.hpp
        LaserScanVisualization.hpp 
        MotionCommandVisualization.hpp 
        RigidBodyStateVisualization.hpp 
        TrajectoryVisualization.hpp 
        WaypointVisualization.hpp 
        SonarGroundDistanceVisualization.hpp 
        SonarBeamVisualization.hpp
        PointcloudVisualization.hpp
        LIBS base ${Boost_SYSTEM_LIBRARY}
)
