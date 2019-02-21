#include "TrajectoryVisualization.hpp"
#include <osg/Geometry>
#include <osg/Geode>

namespace vizkit3d
{

TrajectoryVisualization::TrajectoryVisualization()
    : max_number_of_points(1800), line_width( 1.0 ),
        color(1., 0., 0., 1.), backwardColor(1., 0., 1., 1.), max_velocity(0)
{
    VizPluginRubyMethod(TrajectoryVisualization, base::Vector3d, setColor);
    VizPluginRubyMethod(TrajectoryVisualization, double, setMaxVelocity);
}

TrajectoryVisualization::~TrajectoryVisualization()
{
}

osg::ref_ptr<osg::Node> TrajectoryVisualization::createMainNode()
{
    colorArray = new osg::Vec4Array;
    geom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    geom->setVertexArray(pointsOSG.get());
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINE_STRIP, 0, pointsOSG->size() );
    geom->addPrimitiveSet(drawArrays.get());
    lineWidth = new osg::LineWidth;

    // Add the Geometry (Drawable) to a Geode and
    // return the Geode.
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geom.get() );

    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    return geode;
}


void TrajectoryVisualization::setColor(const base::Vector3d& color)
{
    { boost::mutex::scoped_lock lockit(this->updateMutex);
        this->color = osg::Vec4(color.x(), color.y(), color.z(), 1.0); }
    emit propertyChanged("Color");
    setDirty();
}

void TrajectoryVisualization::setMaxVelocity(double max_velocity)
{
    { boost::mutex::scoped_lock lockit(this->updateMutex);
        this->max_velocity = max_velocity; }
    emit propertyChanged("MaxVelocity");
    setDirty();
}

double TrajectoryVisualization::getMaxVelocity()
{
    return max_velocity;
}


void TrajectoryVisualization::clear()
{
    boost::mutex::scoped_lock lock(updateMutex);
    points.clear();
    setDirty();
}

void TrajectoryVisualization::updateMainNode( osg::Node* node )
{
    osg::StateSet* stategeode = node->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    lineWidth->setWidth(line_width);
    stategeode->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);

    pointsOSG->clear();
    colorArray->clear();
    for(auto it = points.begin(); it != points.end(); it++)
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
        double stepSize, bool showAll)
{
    addSpline(data, color, stepSize, showAll);
}

void TrajectoryVisualization::addSpline(const base::geometry::Spline3& data,
        const osg::Vec4& color, double stepSize, bool showAll)
{
    auto newPoints = convertSpline(data, color, stepSize);

    boost::mutex::scoped_lock lock(updateMutex);
    points.insert(points.end(), newPoints.begin(), newPoints.end());
    if (!showAll)
        enforceMaxPoints();
    setDirty();
}

std::vector<TrajectoryVisualization::Point> TrajectoryVisualization::convertSpline(
    const base::geometry::Spline3& spline, const osg::Vec4& color, double stepSize) const
{
    std::vector<Point> points;
    if (!spline.getSISLCurve())
        return points;

    if (!stepSize)
    {
        // needs a copy as getCurveLength is not const
        base::geometry::Spline3 temp = spline;
        stepSize = (temp.getEndParam() - temp.getStartParam()) / (temp.getCurveLength() / 0.05);
    }

    int size = std::floor((spline.getEndParam() - spline.getStartParam()) / stepSize);
    points.reserve(size);

    for(double param = spline.getStartParam(); param <= spline.getEndParam(); param += stepSize )
    {
        auto splinePoint = spline.getPointAndTangent(param);

        Point p;
        p.point = osg::Vec3(
            splinePoint.first.x(),
            splinePoint.first.y(),
            splinePoint.first.z());

        if(max_velocity > 0)
        {
            double color_multiplier = splinePoint.second.norm()/max_velocity;
            osg::Vec4 gradient_color(color_multiplier, 1 - color_multiplier , 0 ,1);
            p.color = gradient_color;
        }
        else
            p.color = color;

        points.push_back(p);
    }
    return points;
}

void TrajectoryVisualization::enforceMaxPoints()
{
    if (max_number_of_points)
    {
        int diff = points.size() - max_number_of_points;
        if (diff > 0)
            points.erase(points.begin(), points.begin() + diff);
    }
}

void TrajectoryVisualization::updateDataIntern(const base::geometry::Spline3& data)
{
    points = convertSpline(data, color, 0);
    enforceMaxPoints();
}

void TrajectoryVisualization::updateDataIntern(const std::vector<base::Trajectory>& data)
{
    //delete old trajectory
    points.clear();

    for(auto it = data.begin(); it != data.end(); it++)
    {
        auto newPoints = convertSpline(
            it->spline, it->speed >= 0? color : backwardColor, 0);
        points.insert(points.end(), newPoints.begin(), newPoints.end());
    }
    enforceMaxPoints();
}


void TrajectoryVisualization::updateDataIntern( const base::Vector3d& data )
{
    Point p;
    p.point = osg::Vec3(data.x(), data.y(), data.z());
    p.color = color;
    points.push_back(p);
    enforceMaxPoints();
}

void TrajectoryVisualization::setColor(QColor color)
{
    { boost::mutex::scoped_lock lockit(this->updateMutex);
        this->color = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF()); }
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
    { boost::mutex::scoped_lock lockit(this->updateMutex);
        backwardColor = osg::Vec4(c.redF(), c.greenF(), c.blueF(), c.alphaF() ); }
    emit propertyChanged("BackwardColor");
    setDirty();
}

QColor TrajectoryVisualization::getBackwardColor() const
{
    QColor c;
    c.setRgbF(backwardColor.x(), backwardColor.y(), backwardColor.z(), backwardColor.w());
    return c;
}

double TrajectoryVisualization::getLineWidth() const
{
    return line_width;
}

void TrajectoryVisualization::setLineWidth(double line_width)
{
    { boost::mutex::scoped_lock lockit(this->updateMutex);
        this->line_width = line_width; }
    emit propertyChanged("LineWidth");
    setDirty();
}
}
