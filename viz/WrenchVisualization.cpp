#include <iostream>
#include "WrenchVisualization.hpp"
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/Geometry>

using namespace vizkit3d;

WrenchVisualization::WrenchVisualization()
{
}

WrenchVisualization::~WrenchVisualization()
{
}

osg::ref_ptr<osg::Node> WrenchVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries

    double size = 2.0;

    osg::Group* group = new osg::Group;
    
    //force
    osg::MatrixTransform* force_dir = new osg::MatrixTransform;
    osg::ref_ptr<osg::Geode> fg = new osg::Geode();
    osg::ref_ptr<osg::Cylinder> fc = new osg::Cylinder(osg::Vec3f(0, 0, 0.05), 0.01, 0.1);
    osg::ref_ptr<osg::ShapeDrawable> fcd = new osg::ShapeDrawable(fc);
    fcd->setColor(osg::Vec4f(1, 0, 0, 1.0));

    fg->addDrawable(fcd);
    //osg::setColor(osg::Vec4f(1, 0, 0, 1.0), c1g);
    force_dir->addChild(fg);
    group->addChild(force_dir);
    
    //torque
    osg::MatrixTransform* torque_dir = new osg::MatrixTransform;
    osg::ref_ptr<osg::Geode> tg = new osg::Geode();
    osg::ref_ptr<osg::Cylinder> tc = new osg::Cylinder(osg::Vec3f(0, 0, 0.05), 0.01, 0.1);
    osg::ref_ptr<osg::ShapeDrawable> tcd = new osg::ShapeDrawable(tc);
    tcd->setColor(osg::Vec4f(0, 1, 0, 1.0));
    tg->addDrawable(tcd);
    //osg::setColor(osg::Vec4f(0.0, 0.0, 0, 1.0), c2g);
    torque_dir->addChild(tg);
    group->addChild(torque_dir);

    return group;
}

void WrenchVisualization::updateMainNode ( osg::Node* node )
{
    // Update the main node using the data in p->data
    osg::Group* group = node->asGroup();

    osg::MatrixTransform* force_transform = dynamic_cast<osg::MatrixTransform*>(group->getChild(0));
    osg::Vec3d force(state.force.x(), state.force.y(), state.force.z());
    osg::Matrixd S = osg::Matrix::scale(1, 1, force.length());
    osg::Matrixd R;
    R.makeRotate(osg::Vec3d(0,0,1), force);
    force_transform->setMatrix(S * R);

    osg::MatrixTransform* torque_transform = dynamic_cast<osg::MatrixTransform*>(group->getChild(1));
    osg::Vec3d torque(state.torque.x(), state.torque.y(), state.torque.z());
    S = osg::Matrix::scale(1, 1, torque.length());
    R.makeRotate(osg::Vec3d(0,0,1), torque);
    torque_transform->setMatrix(S * R);
}

void WrenchVisualization::updateDataIntern(base::Wrench const& value)
{
    state = value;
}