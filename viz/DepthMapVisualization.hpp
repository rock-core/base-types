#ifndef __DEPTH_MAP_VISUALIZATION_HPP__
#define __DEPTH_MAP_VISUALIZATION_HPP__

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/DepthMap.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <osg/Geode>

namespace vizkit3d
{
    class DepthMapVisualization
        : public vizkit3d::Vizkit3DPlugin<base::samples::DepthMap>
        , public vizkit3d::VizPluginAddType<base::samples::RigidBodyState>
        , boost::noncopyable
    {
    Q_OBJECT
    
    Q_PROPERTY(bool ColorizeAltitude READ isColorizeAltitudeEnabled WRITE setColorizeAltitude)
    Q_PROPERTY(bool ColorizeMagnitude READ isColorizeMagnitudeEnabled WRITE setColorizeMagnitude)
    Q_PROPERTY(double ColorizeInterval READ getColorizeInterval WRITE setColorizeInterval)
    Q_PROPERTY(bool ShowRemission READ isShowRemissionEnabled WRITE setShowRemission)
    Q_PROPERTY(bool ShowSlope READ isShowSlopeEnabled WRITE setShowSlope)
    Q_PROPERTY(QColor defaultFeatureColor READ getDefaultFeatureColor WRITE setDefaultFeatureColor)
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        DepthMapVisualization();
        ~DepthMapVisualization();
        
        Q_INVOKABLE void updateData(const base::samples::DepthMap& data)
        { Vizkit3DPlugin<base::samples::DepthMap>::updateData(data); }
        Q_INVOKABLE void updateDepthMap(const base::samples::DepthMap& data)
        { updateData(data); }
        Q_INVOKABLE void updateData(const base::samples::RigidBodyState& data)
        { Vizkit3DPlugin<base::samples::DepthMap>::updateData(data); }
        Q_INVOKABLE void updatePose(const base::samples::RigidBodyState& data)
        { updateData(data); }
        
    public slots:
        void setColorizeAltitude(bool value);
        bool isColorizeAltitudeEnabled()const;
        void setColorizeMagnitude(bool value);
        bool isColorizeMagnitudeEnabled()const;
        void setColorizeInterval(double value);
        double getColorizeInterval()const;
        void setShowRemission(bool value);
        bool isShowRemissionEnabled() const;
        void setShowSlope(bool value);
        bool isShowSlopeEnabled() const;
        QColor getDefaultFeatureColor();
        void setDefaultFeatureColor(QColor color);

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(base::samples::DepthMap const& sample);
        virtual void updateDataIntern(const base::samples::RigidBodyState& sample);
        
    private:
        base::samples::DepthMap scan_sample;
        Eigen::Vector3d scan_position;
        Eigen::Quaterniond scan_orientation;
        osg::ref_ptr< osg::PositionAttitudeTransform > transformation_node;
        osg::ref_ptr<osg::Geode> scan_node;
        osg::ref_ptr<osg::Geometry> scan_geom;
        osg::ref_ptr<osg::Geometry> slope_geom;
        bool colorize_altitude;
        bool colorize_magnitude;
        double colorize_interval;
        bool show_remission;
        bool show_slope;
        osg::Vec4f default_feature_color;
    };
}
#endif
