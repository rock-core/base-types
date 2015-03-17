#ifndef __SONAR_GROUND_DISTANCE_VISUALIZATION_HPP__
#define __SONAR_GROUND_DISTANCE_VISUALIZATION_HPP__

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <Eigen/Geometry>
#include <base/samples/RigidBodyState.hpp>
#include <osg/Shape>

namespace vizkit3d 
{

class SonarGroundDistanceVisualization : public Vizkit3DPlugin<base::samples::RigidBodyState>
{
        Q_OBJECT
        Q_PROPERTY(double BeamWidth READ getBeamWidth WRITE setBeamWidth)
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SonarGroundDistanceVisualization();	
	virtual ~SonarGroundDistanceVisualization();

        Q_INVOKABLE void updateData( const base::samples::RigidBodyState& state )
        { return Vizkit3DPlugin<base::samples::RigidBodyState>::updateData(state); }
        Q_INVOKABLE void updateRigidBodyState( const base::samples::RigidBodyState& state )
        { return updateData(state); }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
	virtual void updateMainNode(osg::Node* node);
	void updateDataIntern( const base::samples::RigidBodyState& state );
        osg::ref_ptr<osg::Group> createCone();
        
        base::samples::RigidBodyState state;
    
    public slots: 
        void setBeamWidth(double _beam_width){
            beam_width = _beam_width/180.0*M_PI;
            emit propertyChanged("BeamWidth");
        }
        
        double getBeamWidth() const{
            return beam_width/M_PI*180.0; 
        }

    private:
	double beam_width;
	osg::ref_ptr<osg::Node>  body_model;
        osg::ref_ptr<osg::Cone> cone;
};

}
#endif 
