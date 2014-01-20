#ifndef LASERSCANVISUALIZATION_H
#define LASERSCANVISUALIZATION_H

#include <base/samples/LaserScan.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>

namespace osg {
    class Geometry;
}

namespace vizkit3d {

class LaserScanVisualization : public Vizkit3DPlugin<base::samples::LaserScan>, public VizPluginAddType<base::samples::RigidBodyState>
{
    Q_OBJECT

    Q_PROPERTY(bool YForward READ isYForwardModeEnabled WRITE setYForwardMode)
    Q_PROPERTY(bool Colorize READ isColorizeEnabled WRITE setColorize)
    Q_PROPERTY(double ColorizeInterval READ getColorizeInterval WRITE setColorizeInterval)
    Q_PROPERTY(bool ShowPolygon READ isShowPolygonEnabled WRITE setShowPolygon)
    bool mYForward;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LaserScanVisualization();
    ~LaserScanVisualization();
    virtual void updateDataIntern(const base::samples::LaserScan& data);
    virtual void updateDataIntern(const base::samples::RigidBodyState& data);
    virtual void updateMainNode(osg::Node* node);
    virtual osg::ref_ptr< osg::Node > createMainNode();

    Q_INVOKABLE void updateData(const base::samples::LaserScan& data)
    { Vizkit3DPlugin<base::samples::LaserScan>::updateData(data); }
    Q_INVOKABLE void updateLaserScan(const base::samples::LaserScan& data)
    { updateData(data); }
    Q_INVOKABLE void updateData(const base::samples::RigidBodyState& data)
    { Vizkit3DPlugin<base::samples::LaserScan>::updateData(data); }
    Q_INVOKABLE void updatePose(const base::samples::RigidBodyState& data)
    { updateData(data); }

public slots:
    bool isYForwardModeEnabled() const;
    void setYForwardMode(bool enabled);
    void setColorize(bool value);
    bool isColorizeEnabled()const;

    //interval in meter 0 = black , interval = white 
    void setColorizeInterval(double value);
    double getColorizeInterval()const;

    void setShowPolygon(bool value);
    bool isShowPolygonEnabled()const;

protected:
    osg::ref_ptr<osg::Node> cloneCurrentViz();

private:
    base::samples::LaserScan scan;
    Eigen::Vector3d scanPosition;
    Eigen::Quaterniond scanOrientation;
    osg::ref_ptr< osg::PositionAttitudeTransform > transformNode;
    osg::ref_ptr<osg::Geode> scanNode;
    osg::ref_ptr<osg::Geometry> scanGeom;

    bool colorize;
    bool show_polygon;
    double colorize_interval;   // 1/distance 
};

}

#endif // LASERSCANVISUALIZATION_H
