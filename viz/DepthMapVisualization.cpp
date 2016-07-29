#include "DepthMapVisualization.hpp"

#include <iostream>
#include <time.h>

#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/Version>

#include <vizkit3d/Vizkit3DHelper.hpp>
#include <vizkit3d/ColorConversionHelper.hpp>


using namespace vizkit3d;

DepthMapVisualization::DepthMapVisualization() : 
    colorize_altitude(false), colorize_magnitude(false), colorize_interval(1.0), show_remission(false), show_slope(false)
{
    scan_orientation = Eigen::Quaterniond::Identity();
    scan_position.setZero();
    default_feature_color = osg::Vec4f(1.0f,0.f,0.3f,0.8f);
}

DepthMapVisualization::~DepthMapVisualization()
{
}

osg::ref_ptr<osg::Node> DepthMapVisualization::createMainNode()
{
    transformation_node = new osg::PositionAttitudeTransform();
    scan_node = new osg::Geode();
    transformation_node->addChild(scan_node);
    
    scan_geom = new osg::Geometry();

    //setup normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    scan_geom->setNormalArray(normals);
    scan_geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    //set size
    osg::ref_ptr<osg::Point> point = new osg::Point();
    point->setSize(5.0);
    point->setDistanceAttenuation( osg::Vec3(1.0, 1.0, 1.0 ) );
    point->setMinSize( 3.0 );
    point->setMaxSize( 5.0 );
    scan_geom->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );

    //turn on transparacny
    scan_node->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    scan_node->addDrawable(scan_geom);
    
    //setup slope geometry
    slope_geom = new osg::Geometry();
    slope_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    scan_node->addDrawable(slope_geom);

    return transformation_node;
}

void DepthMapVisualization::updateMainNode ( osg::Node* node )
{
    // apply transformation
    transformation_node->setPosition(eigenVectorToOsgVec3(scan_position));
    transformation_node->setAttitude(eigenQuatToOsgQuat(scan_orientation));
    
    // convert depth map to point cloud
    std::vector<Eigen::Vector3d> points;
    scan_sample.convertDepthMapToPointCloud(points, true, false);
    
    //set color binding
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    if(show_remission && !scan_sample.remissions.empty() && scan_sample.remissions.size() != points.size())
    {
        throw std::runtime_error("Remission and depth image sizes are incompatible");
    }
    if(colorize_magnitude || colorize_altitude)
    {
        for(unsigned i = 0; i < points.size(); i++)
        {
	    if(scan_sample.isIndexValid(i))
	    {
		double hue = 0.0;
		if(colorize_altitude)
		    hue = (points[i].z() - std::floor(points[i].z() / colorize_interval) * colorize_interval) / colorize_interval;
		else
		    hue = (points[i].norm() - std::floor(points[i].norm() / colorize_interval) * colorize_interval) / colorize_interval;
		float remission = (show_remission && !scan_sample.remissions.empty()) ? scan_sample.remissions[i] : 0.5;
		osg::Vec4 color( 1.0, 1.0, 1.0, 1.0 );
		hslToRgb(hue, 1.0, remission, color.r(), color.g(), color.b());
		colors->push_back(color);
	    }
        }
        
	#if OSG_MIN_VERSION_REQUIRED(3,1,8)
	    slope_geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
	#else
	    slope_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	    slope_geom->setColorArray(colors);
	#endif
    }
    else if(show_remission && !scan_sample.remissions.empty())
    {
        for(unsigned i = 0; i < points.size(); i++)
        {
	    if(scan_sample.isIndexValid(i))
	    {
	        float re = scan_sample.remissions[i];
	        osg::Vec4f color = default_feature_color * re;
	        color.w() = default_feature_color.w();
		colors->push_back(color);
	    }
        }

	#if OSG_MIN_VERSION_REQUIRED(3,1,8)
	    slope_geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
	#else
	    slope_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	    slope_geom->setColorArray(colors);
	#endif
    }
    else
    {
        colors->push_back(default_feature_color);
	
	#if OSG_MIN_VERSION_REQUIRED(3,1,8)
	    slope_geom->setColorArray(colors, osg::Array::BIND_OVERALL);
	#else
	    slope_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
	    slope_geom->setColorArray(colors);
	#endif
    }

    // convert points to osg
    osg::Vec3Array *scan_vertices = new osg::Vec3Array();
    scan_vertices->reserve(points.size());
    for(unsigned i = 0; i < points.size(); i++)
    {
	if(scan_sample.isIndexValid(i))
	    scan_vertices->push_back(eigenVectorToOsgVec3(points[i]));
    }
    slope_geom->setVertexArray(scan_vertices);

    while(!slope_geom->getPrimitiveSetList().empty())
        slope_geom->removePrimitiveSet(0);

    slope_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,scan_vertices->size()));

    //draw slope geometry
    if(show_slope)
    {
        osg::ref_ptr<osg::Vec3Array> slope_vertices = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec4Array> slope_colors = new osg::Vec4Array();

        base::samples::DepthMap::DepthMatrixMapConst depth_map = scan_sample.getDistanceMatrixMapConst();
	for(unsigned row = 0; row < depth_map.rows()-1; row++)
	{
	    for(unsigned col = 0; col < depth_map.cols(); col++)
	    {
		if(scan_sample.isMeasurementValid(row, col) && scan_sample.isMeasurementValid(row+1, col) &&
		    (std::min(depth_map(row, col), depth_map(row+1, col)) * 1.3) >= std::max(depth_map(row, col), depth_map(row+1, col)))
		{
		    Eigen::Vector3d point1 = points[scan_sample.getIndex(row,col)];
		    Eigen::Vector3d point2 = points[scan_sample.getIndex(row+1,col)];
		    
                    slope_vertices->push_back(eigenVectorToOsgVec3(point1));
                    slope_vertices->push_back(eigenVectorToOsgVec3(point2));
                    
                    Eigen::Vector3d v_diff = point2 - point1;
                    double v_angle = std::abs(std::atan2(v_diff.head<2>().norm(), v_diff.z()));
                    
                    osg::Vec4 color( 1.0, 1.0, 1.0, 1.0 );
                    hslToRgb(v_angle/M_PI, 1.0, 0.5, color.r(), color.g(), color.b());
                    slope_colors->push_back(color);
                    slope_colors->push_back(color);
		}
	    }
	}
        
        #if OSG_MIN_VERSION_REQUIRED(3,1,8)
	    slope_geom->setColorArray(slope_colors, osg::Array::BIND_PER_VERTEX);
        #else
	    slope_geom->setColorArray(slope_colors);
	    slope_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	#endif
        slope_geom->setVertexArray(slope_vertices);
        slope_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, slope_vertices->size()));
    }
}

void DepthMapVisualization::updateDataIntern(base::samples::DepthMap const& sample)
{
    scan_sample = sample;
}

void DepthMapVisualization::updateDataIntern(const base::samples::RigidBodyState& sample)
{
    scan_orientation = sample.orientation;
    scan_position = sample.position;
}

double DepthMapVisualization::getColorizeInterval() const
{
    return colorize_interval;
}

void DepthMapVisualization::setColorizeInterval(double value)
{
    if(value != 0.0)
    {
        colorize_interval = value;
        emit propertyChanged("ColorizeInterval");
    }
}

bool DepthMapVisualization::isColorizeAltitudeEnabled() const
{
    return colorize_altitude;
}

void DepthMapVisualization::setColorizeAltitude(bool value)
{
    colorize_altitude = value;
    emit propertyChanged("ColorizeAltitude");
}

bool DepthMapVisualization::isColorizeMagnitudeEnabled() const
{
    return colorize_magnitude;
}

void DepthMapVisualization::setColorizeMagnitude(bool value)
{
    colorize_magnitude = value;
    emit propertyChanged("ColorizeMagnitude");
}

bool DepthMapVisualization::isShowRemissionEnabled() const
{
    return show_remission;
}

void DepthMapVisualization::setShowRemission(bool value)
{
    show_remission = value;
    emit propertyChanged("ShowRemission");
}

bool DepthMapVisualization::isShowSlopeEnabled() const
{
    return show_slope;
}

void DepthMapVisualization::setShowSlope(bool value)
{
    show_slope = value;
    emit propertyChanged("ShowSlope");
}

QColor DepthMapVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void DepthMapVisualization::setDefaultFeatureColor(QColor color)
{
    default_feature_color.x() = color.redF();
    default_feature_color.y() = color.greenF();
    default_feature_color.z() = color.blueF();
    default_feature_color.w() = color.alphaF();
    emit propertyChanged("defaultFeatureColor");
}
