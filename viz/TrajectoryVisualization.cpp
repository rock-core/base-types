#include "TrajectoryVisualization.hpp"
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>

namespace vizkit 
{

TrajectoryVisualization::TrajectoryVisualization():max_number_of_points(1800)
{
    VizPluginRubyMethod(TrajectoryVisualization, base::Vector3d, setColor);

    // initialize here so that setColor can be called event
    doClear = false;
    colorArray = new osg::Vec4Array;
    geom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    setColor( 1, 0, 0, 1 ); 
}

TrajectoryVisualization::~TrajectoryVisualization()
{
}

osg::ref_ptr<osg::Node> TrajectoryVisualization::createMainNode()
{
    geom->setVertexArray(pointsOSG);
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINE_STRIP, 0, pointsOSG->size() );
    geom->addPrimitiveSet(drawArrays.get());

    // Add the Geometry (Drawable) to a Geode and
    //   return the Geode.
    geode = new osg::Geode;
    geode->addDrawable( geom.get() );

    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    
    return geode;
}

void TrajectoryVisualization::setColor(double r, double g, double b, double a)
{
    color = osg::Vec4( r, g, b, a );

    // set colors
    colorArray->clear();
    colorArray->push_back( color );
    geom->setColorArray( colorArray );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
}


void TrajectoryVisualization::setColor(const base::Vector3d& color)
{
    setColor(color.x(), color.y(), color.z(), 1.0);
}


void TrajectoryVisualization::clear()
{
    doClear = true;
}

void TrajectoryVisualization::updateMainNode( osg::Node* node )
{   
    std::list<Eigen::Vector3d>::const_iterator it = points.begin();

    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    
    pointsOSG->clear();
    for(; it != points.end(); it++) {
	pointsOSG->push_back(osg::Vec3(it->x(), it->y(), it->z()));
    }
    geom->setVertexArray(pointsOSG);
    drawArrays->setCount(pointsOSG->size());
}

void TrajectoryVisualization::updateDataIntern(const base::geometry::Spline3& data)
{
    //needs a copy as getCurveLength is not const
    base::geometry::Spline3 spline = data; 
    
    //delete old trajectory
    points.clear();
    
    if(!data.getSISLCurve())
	return;
    
    //a point every 5 cm
    double stepSize = (spline.getEndParam() - spline.getStartParam()) / (spline.getCurveLength() / 0.05);
    for(double p = spline.getStartParam(); p <= spline.getEndParam(); p += stepSize )
    {
	points.push_back(spline.getPoint(p));
        while(points.size() > max_number_of_points)
            points.pop_front();
    }
}

void TrajectoryVisualization::updateDataIntern( const base::Vector3d& data )
{
    if(doClear)
    {
	points.clear();
	doClear = false;
    }
    Eigen::Vector3d d = data;
    points.push_back(d);
    while(points.size() > max_number_of_points)
        points.pop_front();
}

void TrajectoryVisualization::setColor(QColor color)
{
    setColor( color.redF(), color.greenF(), color.blueF(), color.alphaF() );
    emit propertyChanged("color");
}

QColor TrajectoryVisualization::getColor() const
{
    QColor color;
    color.setRgbF(this->color.x(), this->color.y(), this->color.z(), this->color.w());
    return color;
}

double TrajectoryVisualization::getLineWidth()
{
    return line_width;
}

void TrajectoryVisualization::setLineWidth(double line_width)
{
    this->line_width = line_width;

    osg::StateSet* stateset = geode->getOrCreateStateSet();
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(line_width);
    stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    emit propertyChanged("line_width");
}
}
