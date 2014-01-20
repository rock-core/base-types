rock_vizkit_plugin(base-viz
    PluginLoader.cpp Uncertainty.cpp Vizkit3DHelper.cpp
    MOC LaserScanVisualization.cpp MotionCommandVisualization.cpp RigidBodyStateVisualization.cpp TrajectoryVisualization.cpp WaypointVisualization.cpp SonarGroundDistanceVisualization.cpp PointcloudVisualization.cpp
    HEADERS Uncertainty.hpp Vizkit3DHelper.hpp LaserScanVisualization.hpp MotionCommandVisualization.hpp RigidBodyStateVisualization.hpp TrajectoryVisualization.hpp WaypointVisualization.hpp SonarGroundDistanceVisualization.hpp PointcloudVisualization.hpp
    LIBS base
)
