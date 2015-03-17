#include "WaypointVisualization.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <vizkit3d/Vizkit3DHelper.hpp>

using namespace vizkit3d;

struct WaypointVisualization::Data {
    // Copy of the value given to updateDataIntern.
    // Making a copy is required because of how OSG works
    std::vector<base::Waypoint> data;
};


WaypointVisualization::WaypointVisualization()
    : p(new Data), color(1.0f, 0.0f, 0.0f, 1.0f)
{
}

WaypointVisualization::~WaypointVisualization()
{
    delete p;
}

void WaypointVisualization::setColor(QColor q_color)
{
    color = osg::Vec4(q_color.redF(), q_color.greenF(), 
            q_color.blueF(), q_color.alphaF());
    setDirty();
    emit propertyChanged("Color");
}

QColor WaypointVisualization::getColor() const
{
    QColor q_color;
    q_color.setRgbF(color[0], color[1], color[2], color[3]);
    return q_color;
} 

osg::ref_ptr<osg::Node> WaypointVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> waypoints = new osg::Group();
    
    addWaypoints(waypoints);

    return waypoints;
}

void WaypointVisualization::updateMainNode ( osg::Node* node )
{
    node->asGroup()->removeChildren(0, node->asGroup()->getNumChildren());
    addWaypoints(node->asGroup());
}

void WaypointVisualization::updateDataIntern(std::vector<base::Waypoint> const& data)
{
    p->data = data;
}

void  WaypointVisualization::updateDataIntern ( base::Waypoint const& data ) {
    p->data.clear();
    p->data.push_back(data);
}

void WaypointVisualization::addWaypoints(osg::Group* group) {
    std::vector<base::Waypoint>::iterator it = p->data.begin();
    for(; it != p->data.end(); ++it) {

        // Create sphere.
        osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(osg::Vec3d(0,0,0), 0.1);
        osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sp.get());
        sd->setColor(color);
        
        // Create triangle.
        osg::ref_ptr<osg::Geometry> triangle_geometry = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> triangle_vertices = new osg::Vec3Array();
        triangle_vertices->push_back(osg::Vec3(0.2,0,0));
        triangle_vertices->push_back(osg::Vec3(0,0.6,0));
        triangle_vertices->push_back(osg::Vec3(-0.2,0,0));
        triangle_geometry->setVertexArray(triangle_vertices);
        osg::ref_ptr<osg::DrawElementsUInt> triangle_face = 
                new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
        triangle_face->push_back(0);
        triangle_face->push_back(1);
        triangle_face->push_back(2);
        triangle_geometry->addPrimitiveSet(triangle_face);
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back( color );
        triangle_geometry->setColorArray(colors);
        triangle_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
        
        // Add drawables to the geode.
        osg::ref_ptr<osg::Geode> waypoint_geode = new osg::Geode();
        waypoint_geode->addDrawable(sd.get());
        waypoint_geode->addDrawable(triangle_geometry);
        
        // Create transformer.
        osg::ref_ptr<osg::PositionAttitudeTransform> waypoint_transform = 
                new osg::PositionAttitudeTransform();
        osg::Vec3 position = eigenVectorToOsgVec3(it->position);
        position[2] += 0.01; // Moves the waypoints a little bit above the z=0 plane.
        waypoint_transform->setPosition(position);
        // osg::Quat(0,0,1,heading) != osg::Quat(heading, Vec(0,0,1)).. why?
        // -M_PI/2.0 because rock defines x to be the front axis.
        waypoint_transform->setAttitude(osg::Quat(it->heading-M_PI/2.0, osg::Vec3f(0,0,1)));
        waypoint_transform->addChild(waypoint_geode);
        
        // Adds the waypoints to the main node.
        group->addChild(waypoint_transform);
    }
}
