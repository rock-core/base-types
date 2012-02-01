#ifndef LASERSCANVISUALIZATION_H
#define LASERSCANVISUALIZATION_H

#include <base/samples/laser_scan.h>
#include <base/samples/rigid_body_state.h>
#include <vizkit/Vizkit3DPlugin.hpp>

namespace osg {
    class Geometry;
}

namespace vizkit {

class LaserScanVisualization : public Vizkit3DPlugin<base::samples::LaserScan>, public VizPluginAddType<base::samples::RigidBodyState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LaserScanVisualization();
    virtual void updateDataIntern(const base::samples::LaserScan& data);
    virtual void updateDataIntern(const base::samples::RigidBodyState& data);
    virtual void updateMainNode(osg::Node* node);
    virtual osg::ref_ptr< osg::Node > createMainNode();

private:
    base::samples::LaserScan scan;
    Eigen::Vector3d scanPosition;
    Eigen::Quaterniond scanOrientation;
    osg::ref_ptr< osg::PositionAttitudeTransform > transformNode;
    osg::ref_ptr<osg::Geode> scanNode;
    osg::ref_ptr<osg::Geometry> scanGeom;
};

}

#endif // LASERSCANVISUALIZATION_H
