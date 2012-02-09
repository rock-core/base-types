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
    Q_OBJECT

    Q_PROPERTY(bool YForward READ isYForwardModeEnabled WRITE setYForwardMode);
    bool mYForward;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LaserScanVisualization();
    ~LaserScanVisualization();
    virtual void updateDataIntern(const base::samples::LaserScan& data);
    virtual void updateDataIntern(const base::samples::RigidBodyState& data);
    virtual void updateMainNode(osg::Node* node);
    virtual osg::ref_ptr< osg::Node > createMainNode();

    Q_INVOKABLE void updateLaserScan(const base::samples::LaserScan& data)
    { updateData(data); }
    Q_INVOKABLE void updatePose(const base::samples::RigidBodyState& data)
    { updateData(data); }

public slots:
    bool isYForwardModeEnabled() const;
    void setYForwardMode(bool enabled);

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
