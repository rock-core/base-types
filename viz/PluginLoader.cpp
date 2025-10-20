
#include "PluginLoader.hpp"

#include "LaserScanVisualization.hpp"
#include "WaypointVisualization.hpp"
#include "MotionCommandVisualization.hpp"
#ifdef SISL_FOUND
#include "TrajectoryVisualization.hpp"
#endif
#include "RigidBodyStateVisualization.hpp"
#include "BodyStateVisualization.hpp"
#include "SonarGroundDistanceVisualization.hpp"
#include "PointcloudVisualization.hpp"
#include "SonarBeamVisualization.hpp"
#include "SonarVisualization.hpp"
#include "DepthMapVisualization.hpp"
#include "DistanceImageVisualization.hpp"
#include "OrientedBoundingBoxVisualization.hpp"

namespace vizkit3d {


	/**
	* Returns a list of all available visualization plugins.
	* @return list of plugin names
	*/
	QStringList* QtPluginVizkitBase::getAvailablePlugins() const
	{
		QStringList *pluginNames = new QStringList();
		pluginNames->push_back("WaypointVisualization");
		#ifdef SISL_FOUND
		pluginNames->push_back("TrajectoryVisualization");
		#endif
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
		pluginNames->push_back("OrientedBoundingBoxVisualization");
		return pluginNames;
	}
	
	QObject* QtPluginVizkitBase::createPlugin(QString const& pluginName)
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
		#ifdef SISL_FOUND
		else if (pluginName == "TrajectoryVisualization")
		{
			plugin = new vizkit3d::TrajectoryVisualization();
		}
		#endif
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
		else if (pluginName == "OrientedBoundingBoxVisualization")
		{
		plugin = new vizkit3d::OrientedBoundingBoxVisualization();
		}

		if (plugin)
		{
			return plugin;
		}
		return NULL;
	}

#if QT_VERSION < 0x050000
	Q_EXPORT_PLUGIN2(QtPluginVizkitBase, QtPluginVizkitBase)
#endif
}
