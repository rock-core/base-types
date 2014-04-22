#ifndef PointcloudVisualization_H
#define PointcloudVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/Pointcloud.hpp>

#include <osg/Node>
#include <osg/Geometry>

namespace vizkit3d
{

/**
 * Vizkit plugin to visualize Pointcloudes.
 * 
 */
class PointcloudVisualization : public vizkit3d::Vizkit3DPlugin< base::samples::Pointcloud >
{    
    Q_OBJECT
    Q_PROPERTY(QColor defaultFeatureColor READ getDefaultFeatureColor WRITE setDefaultFeatureColor)
    Q_PROPERTY(double pointSize READ getPointSize WRITE setPointSize)
    
    public:
        PointcloudVisualization();
        
        Q_INVOKABLE void updatePointCloud( const base::samples::Pointcloud& sample )
        { return updateData(sample); }
        Q_INVOKABLE void updateChannelData( const std::vector< base::Vector3d >& channel_data )
        { return updateData(channel_data); }
        
    public slots:
        QColor getDefaultFeatureColor();
        void setDefaultFeatureColor(QColor color);

        double getPointSize();
        void setPointSize(double size);

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const base::samples::Pointcloud& data );
        
    private:
        base::samples::Pointcloud pointCloud;
        osg::Vec4f default_feature_color;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
        bool newPoints;
};

}
#endif
