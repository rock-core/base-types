#include "LaserScanVisualization.hpp"
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <iostream>
#include <vizkit/Vizkit3DHelper.hpp>

using namespace vizkit;

vizkit::LaserScanVisualization::LaserScanVisualization()
    : mYForward(false),colorize(false),show_polygon(true),colorize_interval(5)
{
    scanOrientation = Eigen::Quaterniond::Identity();
    scanPosition.setZero();
}

vizkit::LaserScanVisualization::~LaserScanVisualization()
{
}

void vizkit::LaserScanVisualization::updateDataIntern(const base::samples::LaserScan& data)
{
    scan = data;
}

void vizkit::LaserScanVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    scanOrientation = data.orientation;
    scanPosition = data.position;
}

osg::ref_ptr< osg::Node > vizkit::LaserScanVisualization::createMainNode()
{
    transformNode = new osg::PositionAttitudeTransform();
    scanNode = new osg::Geode();
    transformNode->addChild(scanNode);

    scanGeom = new osg::Geometry();

    //setup normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    scanGeom->setNormalArray(normals);
    scanGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    //set size
    osg::ref_ptr<osg::Point> point = new osg::Point();
    point->setSize(5.0);
    point->setDistanceAttenuation( osg::Vec3(1.0, 1.0, 1.0 ) );
    point->setMinSize( 3.0 );
    point->setMaxSize( 5.0 );
    scanGeom->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );

    //turn on transparacny
    scanNode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    scanNode->addDrawable(scanGeom);
    
    return transformNode;
}

void vizkit::LaserScanVisualization::updateMainNode(osg::Node* node)
{
    transformNode->setPosition(eigenVectorToOsgVec3(scanPosition));
    transformNode->setAttitude(eigenQuatToOsgQuat(scanOrientation));
    
    osg::Vec3Array *scanVertices = new osg::Vec3Array();

    std::vector<Eigen::Vector3d> points;
    scan.convertScanToPointCloud(points);
    if (mYForward)
    {
        base::Quaterniond rot;
        rot = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
        for(std::vector<Eigen::Vector3d>::iterator it = points.begin(); it != points.end(); it++)
            *it = rot * (*it);
    }

    if(colorize)    
    {
        osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
        color->push_back(osg::Vec4(0.0,0.0,0.0,0.0));
        double dist;
        for(std::vector<uint32_t>::const_iterator it = scan.ranges.begin(); it != scan.ranges.end(); it++) 
        {
            dist = 0.001*(*it);
            dist = dist*colorize_interval-((int)(dist*colorize_interval));
            color->push_back(osg::Vec4(dist,dist,dist, 0.8));
        }
        scanGeom->setColorArray(color.get());
        scanGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }
    else
    {
        osg::Vec4Array *colors = new osg::Vec4Array();
        colors->push_back(osg::Vec4(0,0,0.3,0.5));
        colors->push_back(osg::Vec4(1,0,0,1));
        scanGeom->setColorArray(colors);
        scanGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    }

    scanVertices->reserve(points.size()+1);
    scanVertices->push_back(osg::Vec3(0,0,0));

    for(std::vector<Eigen::Vector3d>::const_iterator it = points.begin(); it != points.end(); it++)
	scanVertices->push_back(eigenVectorToOsgVec3(*it));
    scanGeom->setVertexArray(scanVertices);

    while(!scanGeom->getPrimitiveSetList().empty())
	scanGeom->removePrimitiveSet(0);
   
    if(show_polygon)
        scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,scanVertices->size()));
    scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,scanVertices->size()));
}

//display only points  
osg::ref_ptr<osg::Node> LaserScanVisualization::cloneCurrentViz()
{
    //copy hole scene graph and remove PrimitiveSet polygon
    osg::ref_ptr<osg::Group> group = VizPluginBase::cloneCurrentViz()->asGroup();
    osg::ref_ptr<osg::Node> node = group->getChild(0);
    if(!node)
        return group;
    osg::ref_ptr<osg::Group> group2 = node->asGroup();
    if(!group2)
        return group;
    node = group2->getChild(0);
    if(!node)
        return group;
    osg::ref_ptr<osg::Geode> geode = node->asGeode();
    if(!geode)
        return group;
    osg::ref_ptr<osg::Drawable> drawable = geode->getDrawable(0);
    if(!drawable)
        return group;
    osg::ref_ptr<osg::Geometry> geometry = drawable->asGeometry();
    if(!geometry)
        return group;
    if(geometry->getNumPrimitiveSets() == 2)
        geometry->removePrimitiveSet(0);
    return group;
}

bool LaserScanVisualization::isYForwardModeEnabled() const { return mYForward; }
void LaserScanVisualization::setYForwardMode(bool enabled) { mYForward = enabled; }

void LaserScanVisualization::setColorize(bool value){colorize = value;}
bool LaserScanVisualization::isColorizeEnabled()const { return colorize; }

void LaserScanVisualization::setShowPolygon(bool value){show_polygon = value;}
bool LaserScanVisualization::isShowPolygonEnabled()const { return show_polygon; }

void LaserScanVisualization::setColorizeInterval(double value){colorize_interval = 1.0/value;}
double LaserScanVisualization::getColorizeInterval()const { return 1.0/colorize_interval; }
