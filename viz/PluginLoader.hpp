#pragma once

#include <vizkit3d/Vizkit3DPlugin.hpp>

namespace vizkit3d {

class QtPluginVizkitBase : public vizkit3d::VizkitPluginFactory {
	Q_OBJECT
#if QT_VERSION >= 0x050000
	Q_PLUGIN_METADATA(IID "rock.vizkit3d.VizkitPluginFactory")
#endif
	//Q_INTERFACES(vizkit3d::VizkitPluginFactory)

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
