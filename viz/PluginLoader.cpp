#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "LaserScanVisualization.hpp"
#include "WaypointVisualization.hpp"
#include "MotionCommandVisualization.hpp"
#include "TrajectoryVisualization.hpp"
#include "RigidBodyStateVisualization.hpp"
#include "BodyStateVisualization.hpp"
#include "SonarGroundDistanceVisualization.hpp"
#include "PointcloudVisualization.hpp"
#include "SonarBeamVisualization.hpp"
#include "SonarVisualization.hpp"
#include "DepthMapVisualization.hpp"
#include "DistanceImageVisualization.hpp"
#include "RigidBodyStateSE3Visualization.hpp"
#include "WrenchVisualization.hpp"

namespace vizkit3d {
    class QtPluginVizkitBase : public vizkit3d::VizkitPluginFactory {
    private:
    public:

    QtPluginVizkitBase() {
    }

    /**
    * Returns a list of all available visualization plugins.
    * @return list of plugin names
    */
    virtual QStringList* getAvailablePlugins() const
    {
        QStringList *pluginNames = new QStringList();
        pluginNames->push_back("WaypointVisualization");
        pluginNames->push_back("TrajectoryVisualization");
        pluginNames->push_back("MotionCommandVisualization");
        pluginNames->push_back("RigidBodyStateVisualization");
        pluginNames->push_back("BodyStateVisualization");
        pluginNames->push_back("LaserScanVisualization");
        pluginNames->push_back("SonarGroundDistanceVisualization");
        pluginNames->push_back("PointcloudVisualization");
        pluginNames->push_back("SonarBeamVisualization");
        pluginNames->push_back("SonarVisualization");
        pluginNames->push_back("DepthMapVisualization");
        pluginNames->push_back("DistanceImageVisualization");
        pluginNames->push_back("RigidBodyStateSE3Visualization");
        pluginNames->push_back("WrenchVisualization");
        return pluginNames;
    }

    virtual QObject* createPlugin(QString const& pluginName)
    {
        vizkit3d::VizPluginBase* plugin = 0;
        if (pluginName == "WaypointVisualization")
        {
            plugin = new vizkit3d::WaypointVisualization();
        }
        else if (pluginName == "MotionCommandVisualization")
        {
            plugin = new vizkit3d::MotionCommandVisualization();
        }
        else if (pluginName == "TrajectoryVisualization")
        {
            plugin = new vizkit3d::TrajectoryVisualization();
        }
        else if (pluginName == "RigidBodyStateVisualization")
        {
            plugin = new vizkit3d::RigidBodyStateVisualization();
        }
        else if (pluginName == "BodyStateVisualization")
        {
            plugin = new vizkit3d::BodyStateVisualization();
        }
        else if (pluginName == "LaserScanVisualization")
        {
            plugin = new vizkit3d::LaserScanVisualization();
        }
        else if (pluginName == "SonarGroundDistanceVisualization")
        {
            plugin = new vizkit3d::SonarGroundDistanceVisualization();
        }
        else if (pluginName == "PointcloudVisualization")
        {
            plugin = new vizkit3d::PointcloudVisualization();
        }
        else if (pluginName == "SonarBeamVisualization")
        {
            plugin = new vizkit3d::SonarBeamVisualization();
        }
        else if (pluginName == "SonarVisualization")
        {
            plugin = new vizkit3d::SonarVisualization();
        }
        else if (pluginName == "DepthMapVisualization")
        {
            plugin = new vizkit3d::DepthMapVisualization();
        }
        else if (pluginName == "DistanceImageVisualization")
        {
        plugin = new vizkit3d::DistanceImageVisualization();
        }
        else if (pluginName == "RigidBodyStateSE3Visualization")
        {
        plugin = new vizkit3d::RigidBodyStateSE3Visualization();
        }
        else if (pluginName == "WrenchVisualization")
        {
        plugin = new vizkit3d::WrenchVisualization();
        }

        if (plugin) 
        {
            return plugin;
        }
        return NULL;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVizkitBase, QtPluginVizkitBase)
}
