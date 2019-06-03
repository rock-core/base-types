#include "viz/DepthMapVisualization.hpp"
#include "base/samples/DepthMap.hpp"
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>


int main(int argc, char** argv){
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    app.start();

    vizkit3d::Vizkit3DWidget *widget = dynamic_cast<vizkit3d::Vizkit3DWidget*>(app.getWidget());

    //configure plug in
    vizkit3d::DepthMapVisualization plugin;
    plugin.setPluginEnabled(true);
    plugin.setPointSize(5);
    widget->addPlugin(&plugin);

    //Init test data as a single circular scan
    base::samples::DepthMap data;
    data.vertical_size = 1;
    data.horizontal_size = 100;
    data.vertical_interval.push_back(0);
    for(unsigned j = 0; j < data.horizontal_size; j++)
    {
        data.timestamps.push_back(base::Time::now());
        data.horizontal_interval.push_back(base::Angle::fromDeg((360.0/data.horizontal_size)*j).getRad());
    }
    data.horizontal_projection = base::samples::DepthMap::POLAR;
    data.vertical_projection = base::samples::DepthMap::PLANAR;
    data.distances.resize(data.vertical_size * data.horizontal_size, 2);

    //Keep on adding the data
    while(app.isRunning()){
        plugin.updateData(data);
        usleep(500);
    }
}
