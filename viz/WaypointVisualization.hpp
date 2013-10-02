#ifndef WAYPOINTVISUALIZATION_H
#define WAYPOINTVISUALIZATION_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/waypoint.h>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>

#include <Qt/qobject.h>

namespace vizkit3d 
{
    
class WaypointVisualization: public Vizkit3DPlugin<base::Waypoint>
{
    Q_OBJECT
    
    public:
	WaypointVisualization();
	Q_INVOKABLE void updateWp(const base::Waypoint &wp);
	Q_INVOKABLE void updateData(const base::Waypoint &wp)
	{Vizkit3DPlugin<base::Waypoint>::updateData(wp);};
	
	/** Sets the color of the default body model in R, G, B
         *
         * Values must be between 0 and 1
         */
        Q_INVOKABLE void setColor(base::Vector3d const& color);
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
	virtual void updateMainNode( osg::Node* node );
	void updateDataIntern ( const base::Waypoint& data );
	osg::Vec4d color;
	osg::ref_ptr<osg::PositionAttitudeTransform> waypointPosition;
	base::Waypoint waypoint;
	osg::ref_ptr<osg::Geode> geode;
	osg::ref_ptr<osg::Sphere> sphere;
	osg::ref_ptr<osg::ShapeDrawable> sphereDrawable;
};

}
#endif // WAYPOINTVISUALIZATION_H
