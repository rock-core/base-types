#ifndef TRAJECTORYVISUALISATION_H
#define TRAJECTORYVISUALISATION_H
#include <Eigen/Geometry>
#include <vector>
#include <osg/Geometry>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <base/geometry/spline.h>

namespace vizkit 
{

class TrajectoryVisualization: public Vizkit3DPlugin<base::Vector3d>, public VizPluginAddType<base::geometry::Spline3>
{
    Q_OBJECT

    public:
	TrajectoryVisualization();
        ~TrajectoryVisualization();
        void setColor(const base::Vector3d& color); 
	void setColor(double r, double g, double b, double a);
	void clear();

    protected:
	virtual osg::ref_ptr<osg::Node> createMainNode();
	virtual void updateMainNode( osg::Node* node );
	void updateDataIntern( const  base::Vector3d& data );
	void updateDataIntern(const base::geometry::Spline3& data);
	
    private:
	osg::Vec4Array *color2; 
	bool doClear;

	std::vector<Eigen::Vector3d> points;
	osg::ref_ptr<osg::Vec3Array> pointsOSG;
	osg::ref_ptr<osg::DrawArrays> drawArrays;
	osg::ref_ptr<osg::Geometry> geom;
};

}
#endif // TRAJECTORYVISUALISATION_H
