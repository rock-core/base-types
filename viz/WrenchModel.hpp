#ifndef WRENCHMODEL_HPP
#define WRENCHMODEL_HPP

#include <osg/Switch>
#include <base/samples/Wrench.hpp>

#include <osgViz/OsgViz.hpp>
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/modules/viz/Primitives/Primitives/ArrowNode.h>

class WrenchModel : public osg::Switch {

public:
    WrenchModel(double resolution = 1.0);
    
    void update(const base::Wrench& wrench);

    void seperateAxes(bool val = true) {
        seperateAxesForce(val);
        seperateAxesTorque(val);
    }
    void seperateAxesForce(bool val=true) {
        show_seperate_axes_force = val;
        setChildValue(force_node, !val);
        setChildValue(force_axes_group, val);
    }
    void seperateAxesTorque(bool val=true) {
        show_seperate_axes_torque = val;
        setChildValue(torque_group, !val);
        setChildValue(torque_axes_group, val);
    }
    void setResolution(double res) {
        resolution = res;
    }

protected:
    //osg::ref_ptr<osg::Switch> model;

    osg::ref_ptr<osg::PositionAttitudeTransform> createCircularArrow(const osg::Vec4f& color);
    void buildGeometry();

    bool show_seperate_axes_force;
    bool show_seperate_axes_torque;
    double resolution;

    osg::ref_ptr<osgviz::Object> force_node;
    osg::ref_ptr<osg::Group> torque_group;
    osg::ref_ptr<osg::Switch> force_axes_group, torque_axes_group;

    std::shared_ptr<osgviz::PrimitivesFactory> primitivesfactory;

private:


};

#endif