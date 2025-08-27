rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

include(RockQt)

unset(OPTIONAL_HPP)
unset(OPTIONAL_MOC)
unset(OPTIONAL_CPP)
if(SISL_FOUND)
    list(APPEND OPTIONAL_HPP
        TrajectoryVisualization.hpp
    )
    list(APPEND OPTIONAL_MOC
        TrajectoryVisualization.hpp
    )
    list(APPEND OPTIONAL_CPP
        TrajectoryVisualization.cpp
    )
endif()

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
        ${OPTIONAL_CPP}
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
        ${OPTIONAL_MOC}
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
        ${OPTIONAL_HPP}
    DEPS base-types
    LIBS ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG base-logging PrimitivesFactory
)
if(SISL_FOUND)
    if(TARGET base-viz)
        target_compile_definitions(base-viz PRIVATE SISL_FOUND)
    endif()
    if(TARGET base-viz-qt5)
        target_compile_definitions(base-viz-qt5 PRIVATE SISL_FOUND)
    endif()
endif(SISL_FOUND)
