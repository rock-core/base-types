#pragma once

#include <vizkit3d/Vizkit3DPlugin.hpp>

namespace vizkit3d {

class QtPluginVizkitBase : public vizkit3d::VizkitPluginFactory {
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "QtPluginVizkitBase")
	//Q_INTERFACES(VizkitPluginFactory_iid)

 private:
 public:
	
	QtPluginVizkitBase() {
	}

	/**
	* Returns a list of all available visualization plugins.
	* @return list of plugin names
	*/
	virtual QStringList* getAvailablePlugins() const;
	virtual QObject* createPlugin(QString const& pluginName);
};

}  // namespace vizkit3d
