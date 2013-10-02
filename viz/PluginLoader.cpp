#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "LaserScanVisualization.hpp"
#include "WaypointVisualization.hpp"
#include "MotionCommandVisualization.hpp"
#include "TrajectoryVisualization.hpp"
#include "RigidBodyStateVisualization.hpp"
#include "SonarGroundDistanceVisualization.hpp"
#include "PointcloudVisualization.hpp"

namespace vizkit3d {
    class QtPluginVizkit : public vizkit3d::VizkitPluginFactory {
    private:
    public:
	
	QtPluginVizkit() {
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
	    pluginNames->push_back("LaserScanVisualization");
	    pluginNames->push_back("SonarGroundDistanceVisualization");
	    pluginNames->push_back("GridVisualization");
	    pluginNames->push_back("PointcloudVisualization");
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

	    if (plugin) 
	    {
		return plugin;
	    }
	    return NULL;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVizkit, QtPluginVizkit)
}
