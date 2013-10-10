#ifndef __RIGID_BODY_STATE_VISUALIZATION_HPP__
#define __RIGID_BODY_STATE_VISUALIZATION_HPP__

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <Eigen/Geometry>
#include <base/samples/RigidBodyState.hpp>

namespace vizkit3d 
{

class RigidBodyStateVisualization : public Vizkit3DPlugin<base::samples::RigidBodyState>
{
        Q_OBJECT
        Q_PROPERTY(double size READ getSize WRITE setSize)
        Q_PROPERTY(double sphereSize READ getMainSphereSize WRITE setMainSphereSize)
        Q_PROPERTY(bool displayCovariance READ isCovarianceDisplayed WRITE displayCovariance)
        Q_PROPERTY(bool displayCovarianceWithSamples READ isCovarianceDisplayedWithSamples WRITE displayCovarianceWithSamples)
        Q_PROPERTY(bool forcePositionDisplay READ isPositionDisplayForced WRITE setPositionDisplayForceFlag)
        Q_PROPERTY(bool forceOrientationDisplay READ isOrientationDisplayForced WRITE setOrientationDisplayForceFlag)

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	RigidBodyStateVisualization(QObject* parent = NULL);
	virtual ~RigidBodyStateVisualization();

        Q_INVOKABLE void updateData( const base::samples::RigidBodyState& state )
        { return Vizkit3DPlugin<base::samples::RigidBodyState>::updateData(state); }
        Q_INVOKABLE void updateRigidBodyState( const base::samples::RigidBodyState& state )
        { return updateData(state); }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
	virtual void updateMainNode(osg::Node* node);
	void updateDataIntern( const base::samples::RigidBodyState& state );
        base::samples::RigidBodyState state;
    
    public slots: 
        bool isPositionDisplayForced() const;
        void setPositionDisplayForceFlag(bool flag);
        bool isOrientationDisplayForced() const;
        void setOrientationDisplayForceFlag(bool flag);

        double getSize() const;
        void setSize(double size);

        void resetModel(double size);
	void resetModelSphere(double size);
	
        void loadModel(std::string const& path);

        /** When using the default body, sets the size of the main sphere,
         * relative to the size of the complete object
         *
         * The default is 0.1
         */
        void setMainSphereSize(double size);

        /** When using one of the default bodies, returns the size of the main
         * sphere, relative to the size of the complete object
         *
         * The default is 0.1
         */
        double getMainSphereSize() const;

        void displayCovariance(bool enable);
        bool isCovarianceDisplayed() const;
        void displayCovarianceWithSamples(bool enable);
        bool isCovarianceDisplayedWithSamples() const;

        /** Sets the color of the default body model in R, G, B
         *
         * Values must be between 0 and 1
         *
         * If you call it after the plugin got attached, call resetModel to
         * apply the new color
         */
        void setColor(base::Vector3d const& color);
	
	void setColor(const osg::Vec4d& color, osg::Geode* geode);
	
    private:
        bool covariance;
        bool covariance_with_samples;
        base::Vector3d color;
        double total_size;
        double main_size;

        enum BODY_TYPES
        { BODY_NONE, BODY_SIMPLE, BODY_SPHERE, BODY_CUSTOM_MODEL };

	osg::Vec3d pos;
	osg::Quat orientation;
        BODY_TYPES body_type;
	osg::ref_ptr<osg::Node>  body_model;
        osg::ref_ptr<osg::Group> createSimpleBody(double size);
	osg::ref_ptr<osg::Group> createSimpleSphere(double size);

        bool forcePositionDisplay;
        bool forceOrientationDisplay;

};

}
#endif // ROBOT_H
