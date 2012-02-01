#ifndef WAYPOINTVISUALIZATION_H
#define WAYPOINTVISUALIZATION_H

#include <vizkit/VizPlugin.hpp>
#include <base/waypoint.h>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>

#include <Qt/qobject.h>

namespace vizkit 
{
    
class WaypointVisualization: public Vizkit3DPlugin<base::Waypoint>
{
    Q_OBJECT
    
    public:
	WaypointVisualization();
	Q_INVOKABLE void updateWp(const base::Waypoint &wp);
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
	virtual void updateMainNode( osg::Node* node );
	void updateDataIntern ( const base::Waypoint& data );
	osg::ref_ptr<osg::PositionAttitudeTransform> waypointPosition;
	base::Waypoint waypoint;
	osg::ref_ptr<osg::Vec3Array> pointsOSG;
	osg::ref_ptr<osg::DrawArrays> drawArrays;
	osg::ref_ptr<osg::Geometry> geom;
};

}
#endif // WAYPOINTVISUALIZATION_H
