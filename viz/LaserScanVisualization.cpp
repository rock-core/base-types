#include "LaserScanVisualization.hpp"
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <iostream>
#include <vizkit/Vizkit3DHelper.hpp>

using namespace vizkit;

vizkit::LaserScanVisualization::LaserScanVisualization()
    : mYForward(false)
{
    VizPluginRubyAdapter(LaserScanVisualization, base::samples::LaserScan, LaserScan)
    VizPluginRubyAdapter(LaserScanVisualization, base::samples::RigidBodyState, Pose)
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
    
    //Set color
    osg::Vec4Array *colors = new osg::Vec4Array();
    colors->push_back(osg::Vec4(0,0,0.3,0.5));
    colors->push_back(osg::Vec4(1,0,0,1));
    scanGeom->setColorArray(colors);
    scanGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    //turn on transparacny
    scanNode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    
    scanNode->addDrawable(scanGeom);
    
    return transformNode;
}

bool LaserScanVisualization::isYForwardModeEnabled() const { return mYForward; }
void LaserScanVisualization::setYForwardMode(bool enabled)
{
    std::cout << "SET " << enabled << std::endl;
    mYForward = enabled;
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

    scanVertices->reserve(points.size() + 1);
    
    //origin of scan
    scanVertices->push_back(osg::Vec3(0,0,0));
    
    for(std::vector<Eigen::Vector3d>::const_iterator it = points.begin(); it != points.end(); it++)
    {
	scanVertices->push_back(eigenVectorToOsgVec3(*it));
    }
    scanGeom->setVertexArray(scanVertices);

    while(!scanGeom->getPrimitiveSetList().empty())
	scanGeom->removePrimitiveSet(0);
    
    scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,scanVertices->size()));

    scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,scanVertices->size()));
}

