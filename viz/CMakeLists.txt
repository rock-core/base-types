rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

if(SISL_FOUND)
    list(APPEND OPTIONAL_HPP
        TrajectoryVisualization.hpp
    )
    list(APPEND OPTIONAL_CPP
        TrajectoryVisualization.cpp
    )
endif()

set(BASE_VIZ_SOURCES
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
    )

set(BASE_VIZ_HEADERS
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
    )

set(BASE_VIZ_MOC
    PluginLoader.hpp
        BodyStateVisualization.hpp
        DepthMapVisualization.hpp
        DistanceImageVisualization.hpp
        LaserScanVisualization.hpp
        MotionCommandVisualization.hpp
        PointcloudVisualization.hpp
        RigidBodyStateVisualization.hpp
        SonarBeamVisualization.hpp
        SonarGroundDistanceVisualization.hpp
        SonarVisualization.hpp
        TrajectoryVisualization.hpp
        WaypointVisualization.hpp
        OrientedBoundingBoxVisualization.hpp
    )

if (ROCK_QT_VERSION_4)
rock_vizkit_plugin(base-viz
        ${BASE_VIZ_SOURCES}
    HEADERS
        ${BASE_VIZ_HEADERS}
    MOC
        ${BASE_VIZ_MOC}
    DEPS base-types
    LIBS ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG base-logging PrimitivesFactory
)
endif()

if (ROCK_QT_VERSION_5)
rock_vizkit_plugin_qt5(base-viz-qt5
        ${BASE_VIZ_SOURCES}
    HEADERS
        ${BASE_VIZ_HEADERS}
    MOC5
        ${BASE_VIZ_MOC}
    DEPS base-types
    LIBS ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG base-logging PrimitivesFactory
)
endif()
