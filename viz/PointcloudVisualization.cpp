#include "PointcloudVisualization.hpp"
#include <osg/Geode>

namespace vizkit
{

PointcloudVisualization::PointcloudVisualization()
{
    newPoints = false;
    default_feature_color = osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
}

/**
 * Creates the main node and attachs the point cloud.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > PointcloudVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
    // set up point cloud
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    color = new osg::Vec4Array;
    pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
    pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    
    return mainNode;
}

QColor PointcloudVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void PointcloudVisualization::setDefaultFeatureColor(QColor color)
{
    default_feature_color.x() = color.redF();
    default_feature_color.y() = color.greenF();
    default_feature_color.z() = color.blueF();
    default_feature_color.w() = color.alphaF();
    emit propertyChanged("defaultFeatureColor");
}

/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void PointcloudVisualization::updateDataIntern(const base::samples::Pointcloud& data)
{
    pointCloud = data;
    newPoints = true;
}

/**
 * Main callback method of osg to update all drawings.
 * The method updates the position if there is a new one and
 * adds new sonar beams if there are new ones. If a wall 
 * estimatior is connected the position of the wall will 
 * be updated too.
 * 
 * @param node osg main node
 */
void PointcloudVisualization::updateMainNode(osg::Node* node)
{
    if(newPoints)
    {
        newPoints = false;
        pointsOSG->clear();
        color->clear();
        int i=0;
        for(std::vector<base::Vector3d>::const_iterator pos = pointCloud.points.begin(); pos != pointCloud.points.end(); pos++)
        {
            i++;
            osg::Vec3d vec(pos->x(), pos->y(), pos->z());
            pointsOSG->push_back(vec);
            if(pointCloud.colors.size() == pointCloud.points.size()){
                osg::Vec4f v = osg::Vec4f(pointCloud.colors[i][0]/255.0, pointCloud.colors[i][1]/255.0, pointCloud.colors[i][2]/255.0, 1.0f);
                color->push_back(v);
            }else{
                printf("Sizes missmatch %i %i\n",pointCloud.colors.size() ,pointCloud.points.size());
                color->push_back(default_feature_color);
            }
            
        }
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        pointGeom->setColorArray(color);
    }
}

}
