#include <iostream>
#include "WrenchVisualization.hpp"

using namespace vizkit3d;

WrenchVisualization::WrenchVisualization()
    : text_size(0.1)
    , show_seperate_axes_force(false)
    , show_seperate_axes_torque(false)
    , resolution(0.1)
{
}

WrenchVisualization::~WrenchVisualization()
{
}

osg::ref_ptr<osg::Node> WrenchVisualization::createMainNode()
{
    wrench_model = new WrenchModel(resolution);
    return wrench_model;
}

double WrenchVisualization::getTextSize() const
{
    return text_size;
}

void WrenchVisualization::setTextSize(double size)
{
    text_size = size;
    text->setCharacterSize(text_size);
    emit propertyChanged("textSize");
    setDirty();
}

void WrenchVisualization::updateMainNode ( osg::Node* node )
{
    // WrenchModel* wrench_model = dynamic_cast<WrenchModel*>(node);
    // if (wrench_model) {
    //     wrench_model->seperateAxes(show_seperate_axes);
    //     wrench_model->setResolution(resolution);
    //     wrench_model->update(state);
    // }
    if (state.isValid())
        wrench_model->update(state);
}

void WrenchVisualization::updateDataIntern(const base::Wrench& value)
{
    state.force = value.force;
    state.torque = value.torque;
}

void WrenchVisualization::updateDataIntern(const base::samples::Wrench& value)
{
    state = value;
}

//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(WrenchVisualization)

