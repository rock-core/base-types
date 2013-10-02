#include "SonarGroundDistanceVisualization.hpp"
#include "Uncertainty.hpp"
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osg/Material>

using namespace vizkit3d;

SonarGroundDistanceVisualization::SonarGroundDistanceVisualization():
    beam_width(3.0/180.0*M_PI)
{
}

SonarGroundDistanceVisualization::~SonarGroundDistanceVisualization()
{
}

osg::ref_ptr<osg::Group> SonarGroundDistanceVisualization::createCone()
{   
    osg::ref_ptr<osg::Group> group = new osg::Group();
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    cone = new osg::Cone(osg::Vec3f(0,0,state.position.z()*-0.75), state.position.z(),(sin(beam_width)*state.position.z()));
    osg::ref_ptr<osg::ShapeDrawable> spd = new osg::ShapeDrawable(cone);
    spd->setColor(osg::Vec4f(0,0,0, 0.5));
    geode->addDrawable(spd);
    group->addChild(geode);

    return group;
}

osg::ref_ptr<osg::Node> SonarGroundDistanceVisualization::createMainNode()
{
    osg::Group* group = new osg::Group;
    osg::PositionAttitudeTransform* body_pose = new osg::PositionAttitudeTransform();
    if (!body_model)
        body_model = createCone();
    body_pose->addChild(body_model);
    group->addChild(body_pose);
    return group;
}

void SonarGroundDistanceVisualization::updateMainNode(osg::Node* node)
{
    cone->set(osg::Vec3f(0,0,state.position.z()*-0.75),sin(beam_width)*state.position.z(),state.position.z());
}

void SonarGroundDistanceVisualization::updateDataIntern( const base::samples::RigidBodyState& state )
{
    this->state = state;
}


