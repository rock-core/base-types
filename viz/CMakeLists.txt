rock_find_qt5(Widgets)
if(SISL_FOUND)
    list(APPEND OPTIONAL_HPP
        TrajectoryVisualization.hpp
    )
    list(APPEND OPTIONAL_CPP
        TrajectoryVisualization.cpp
    )
endif()

rock_vizkit_plugin(base-viz
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
        ${OPTIONAL_CPP}
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
        ${OPTIONAL_HPP}
    DEPS base-types Qt5::Widgets
    LIBS ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG base-logging
)
