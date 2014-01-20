#include "WaypointVisualization.hpp"
#include <iostream>

namespace vizkit3d 
{

WaypointVisualization::WaypointVisualization()
{    
    color = osg::Vec4( 1, 0, 0, 1 );
    
    waypoint.tol_position = 0.0;
}

void WaypointVisualization::updateWp(const base::Waypoint& wp)
{
    updateData(wp);
}

osg::ref_ptr< osg::Node > WaypointVisualization::createMainNode()
{
    geode = new osg::Geode;

//     osg::StateSet* stategeode = geode->getOrCreateStateSet();
//     stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    waypointPosition = new osg::PositionAttitudeTransform();
    
    sphere = new osg::Sphere(osg::Vec3f(0,0,0),0.3);
    sphere->setDataVariance(osg::Object::DYNAMIC);
    sphereDrawable = new osg::ShapeDrawable(sphere);
    sphereDrawable->setColor(color);
    sphereDrawable->setDataVariance(osg::Object::DYNAMIC);
    
    return geode;
}

void WaypointVisualization::setColor(const base::Vector3d& color)
{
    this->color = osg::Vec4d(color.x(), color.y(), color.z(), 1);
    setDirty();
}

void WaypointVisualization::updateDataIntern ( const base::Waypoint& data )
{
    waypoint = data;
}

void WaypointVisualization::updateMainNode( osg::Node* node )
{
    sphere->setRadius(waypoint.tol_position);
    sphere->setCenter(osg::Vec3(waypoint.position.x(), waypoint.position.y(), waypoint.position.z()));
    
    sphereDrawable->setColor(color);
    sphereDrawable->dirtyDisplayList();
    sphereDrawable->dirtyBound();
    
    geode->removeDrawable(sphereDrawable);
    geode->addDrawable(sphereDrawable);
    geode->dirtyBound();
}

}
