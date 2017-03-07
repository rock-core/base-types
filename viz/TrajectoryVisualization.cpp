#include "TrajectoryVisualization.hpp"
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>

namespace vizkit3d 
{

TrajectoryVisualization::TrajectoryVisualization()
    : doClear(false), max_number_of_points(1800), line_width( 1.0 ), color(1., 0., 0., 1.), backwardColor(1., 0., 1., 1.)
{
    VizPluginRubyMethod(TrajectoryVisualization, base::Vector3d, setColor);
}

TrajectoryVisualization::~TrajectoryVisualization()
{
}

osg::ref_ptr<osg::Node> TrajectoryVisualization::createMainNode()
{
    doClear = false;
    colorArray = new osg::Vec4Array;
    geom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    geom->setVertexArray(pointsOSG.get());
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


void TrajectoryVisualization::setColor(const base::Vector3d& color)
{
    this->color = osg::Vec4(color.x(), color.y(), color.z(), 1.0);
    emit propertyChanged("Color");
    setDirty();
}


void TrajectoryVisualization::clear()
{
    doClear = true;
}

void TrajectoryVisualization::updateMainNode( osg::Node* node )
{   

    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    
    pointsOSG->clear();
    colorArray->clear();
    for(std::deque<Point>::iterator it = points.begin(); it != points.end(); it++)
    {
        pointsOSG->push_back(it->point);
        colorArray->push_back(it->color);
    }
    
    geom->setVertexArray(pointsOSG);
    drawArrays->setCount(pointsOSG->size());
    geom->setColorArray(colorArray);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
}

void TrajectoryVisualization::addSpline(const base::geometry::Spline3& data,
                                        const osg::Vec4& color)
{
    //needs a copy as getCurveLength is not const
    base::geometry::Spline3 spline = data; 
    
    if(!data.getSISLCurve())
	return;
    
    //a point every 5 cm
    double stepSize = (spline.getEndParam() - spline.getStartParam()) / (spline.getCurveLength() / 0.05);
    for(double param = spline.getStartParam(); param <= spline.getEndParam(); param += stepSize )
    {
        const Eigen::Vector3d splinePoint = spline.getPoint(param);
        Point p;
        p.point = osg::Vec3(splinePoint.x(), splinePoint.y(), splinePoint.z());
        p.color = color;
        points.push_back(p);
    }

    while(points.size() > max_number_of_points)
        points.pop_front();
}

void TrajectoryVisualization::updateDataIntern(const base::geometry::Spline3& data)
{
    //delete old trajectory
    points.clear();

    addSpline(data, color);
}

void TrajectoryVisualization::updateDataIntern(const std::vector<base::Trajectory>& data)
{
    //delete old trajectory
    points.clear();

    for(std::vector<base::Trajectory>::const_iterator it = data.begin(); it != data.end(); it++)
    {
        addSpline(it->spline, it->speed >= 0? color : backwardColor);
    }
}


void TrajectoryVisualization::updateDataIntern( const base::Vector3d& data )
{
    if(doClear)
    {
        points.clear();
        doClear = false;
    }
    Point p;
    p.point = osg::Vec3(data.x(), data.y(), data.z());
    p.color = color;
    points.push_back(p);
    while(points.size() > max_number_of_points)
        points.pop_front();
}

void TrajectoryVisualization::setColor(QColor color)
{
    this->color = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("Color");
    setDirty();
}

QColor TrajectoryVisualization::getColor() const
{
    QColor c;
    c.setRgbF(color.x(), color.y(), color.z(), color.w());
    return c;
}

void TrajectoryVisualization::setBackwardColor(QColor c)
{
    backwardColor = osg::Vec4(c.redF(), c.greenF(), c.blueF(), c.alphaF() );
    emit propertyChanged("BackwardColor");
    setDirty();
}

QColor TrajectoryVisualization::getBackwardColor() const
{
    QColor c;
    c.setRgbF(backwardColor.x(), backwardColor.y(), backwardColor.z(), backwardColor.w());
    return c;
}

double TrajectoryVisualization::getLineWidth()
{
    return line_width;
}

void TrajectoryVisualization::setLineWidth(double line_width)
{
    this->line_width = line_width;
    if(geode)
    {
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        osg::LineWidth* linewidth = new osg::LineWidth();
        linewidth->setWidth(line_width);
        stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    }
    emit propertyChanged("LineWidth");
}
}
