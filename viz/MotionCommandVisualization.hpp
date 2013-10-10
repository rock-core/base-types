#ifndef MOTIONCOMMANDVISUALIZATION_H
#define MOTIONCOMMANDVISUALIZATION_H
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/commands/Motion2D.hpp>
#include <base/Pose.hpp>
#include <osg/Shape>
#include <Eigen/Geometry>
#include <osg/Drawable>

namespace vizkit3d 
{

class MotionCommandVisualization : public Vizkit3DPlugin<base::commands::Motion2D>, public VizPluginAddType<base::Pose>
{
    Q_OBJECT
    Q_PROPERTY(FrontAxis frontAxis READ getFrontAxis WRITE setFrontAxis)   
    Q_ENUMS(FrontAxis)

    public:
	MotionCommandVisualization();	
        ~MotionCommandVisualization();
        
        enum FrontAxis {FrontAxisX, FrontAxisY};

        Q_INVOKABLE void updateData(const base::commands::Motion2D& data)
        { Vizkit3DPlugin<base::commands::Motion2D>::updateData(data); }
        Q_INVOKABLE void updateMotionCommand(const base::commands::Motion2D& data)
        { updateData(data); }
        Q_INVOKABLE void updateData(const base::Pose& data)
        { Vizkit3DPlugin<base::commands::Motion2D>::updateData(data); }
        Q_INVOKABLE void updatePose(const base::Pose& data)
        { updateData(data); }

    public slots:
        void setFrontAxis(FrontAxis front_axis);
        FrontAxis getFrontAxis();

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
	void updateDataIntern ( const base::commands::Motion2D& data );
        void updateDataIntern ( const base::Pose& data );

    private:
	double tv;
	double rv;
	osg::Vec3 robotPosition;
	osg::Quat robotOrientation;
	osg::ref_ptr<osg::Cylinder> motionPointer;
	osg::ref_ptr<osg::Cone> motionPointerHead;
	osg::ref_ptr<osg::Vec3Array> pointsOSG;
	osg::ref_ptr<osg::DrawArrays> drawArrays;
	osg::ref_ptr<osg::Geometry> geom;
	osg::ref_ptr<osg::PositionAttitudeTransform> arrowRotation;
	osg::ref_ptr<osg::PositionAttitudeTransform> positionTransformation;
    FrontAxis mFrontAxis; 
    void drawRotation();
};

}
#endif // MOTIONCOMMANDVISUALIZATION_H
