#ifndef __RIGID_BODY_STATE_SE3_VISUALIZATION_HPP__
#define __RIGID_BODY_STATE_SE3_VISUALIZATION_HPP__

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <Eigen/Geometry>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <osgViz/OsgViz.hpp>
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>

#include <osg/Image>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include "WrenchModel.hpp"

namespace osgFX
{
    class BumpMapping;
}

namespace vizkit3d 
{

class RigidBodyStateSE3Visualization : public Vizkit3DPlugin<base::samples::RigidBodyStateSE3>
{
        Q_OBJECT
        Q_PROPERTY(double size READ getSize WRITE setSize)
        Q_PROPERTY(double sphereSize READ getMainSphereSize WRITE setMainSphereSize)
        Q_PROPERTY(double textSize READ getTextSize WRITE setTextSize)
        Q_PROPERTY(bool forcePositionDisplay READ isPositionDisplayForced WRITE setPositionDisplayForceFlag)
        Q_PROPERTY(bool forceOrientationDisplay READ isOrientationDisplayForced WRITE setOrientationDisplayForceFlag)
        Q_PROPERTY(QString modelPath READ getModelPath WRITE loadModel)
        Q_PROPERTY(bool showPose READ isPoseDisplayed WRITE setPoseDisplayed)
        Q_PROPERTY(bool showVelocity READ isVelocityDisplayed WRITE setVelocityDisplayed)
        Q_PROPERTY(bool showAcceleration READ isAccelerationDisplayed WRITE setAccelerationDisplayed)
        Q_PROPERTY(bool showWrench READ isWrenchDisplayed WRITE setWrenchDisplayed)
        Q_PROPERTY(bool showSeperateAxes READ isSeperateAxes WRITE setSeperateAxes)
        Q_PROPERTY(double resolution READ getResolution WRITE setResolution)

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	RigidBodyStateSE3Visualization(QObject* parent = NULL);
	virtual ~RigidBodyStateSE3Visualization();

        Q_INVOKABLE void updateData( const base::samples::RigidBodyStateSE3& state )
        { return Vizkit3DPlugin<base::samples::RigidBodyStateSE3>::updateData(state); }
        Q_INVOKABLE void updateRigidBodyStateSE3( const base::samples::RigidBodyStateSE3& state )
        { return updateData(state); }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
	    virtual void updateMainNode(osg::Node* node);
	    void updateDataIntern( const base::samples::RigidBodyStateSE3& state );
        base::samples::RigidBodyStateSE3 state;
    
    public slots: 
        bool isPositionDisplayForced() const;
        void setPositionDisplayForceFlag(bool flag);
        bool isOrientationDisplayForced() const;
        void setOrientationDisplayForceFlag(bool flag);

        bool isPoseDisplayed() const;
        void setPoseDisplayed(bool flag);

        bool isVelocityDisplayed() const;
        void setVelocityDisplayed(bool flag);

        bool isAccelerationDisplayed() const;
        void setAccelerationDisplayed(bool flag);

        bool isWrenchDisplayed() const;
        void setWrenchDisplayed(bool flag);

        double getSize() const;
        void setSize(double size);

        void resetModel(double size);
	    void resetModelSphere(double size);
        void updateModel(double size);
	
        QString getModelPath() const;
        void loadModel(std::string const& path);
        void loadModel(QString const& path);

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

        /** Sets the text size relative to the size of the complete object.
         * If text size is positive, the name of the source frame is rendered in the visualization.
         * The default is 0.0
         */
        void setTextSize(double size);
        double getTextSize() const;

        /** Sets the color of the default body model in R, G, B
         *
         * Values must be between 0 and 1
         *
         * If you call it after the plugin got attached, call resetModel to
         * apply the new color
         */
        void setColor(base::Vector3d const& color);
	
	    void setColor(const osg::Vec4d& color, osg::Geode* geode);
	
        void setTexture(QString const& path);
        void setTexture(std::string const& path);
        void clearTexture();
        void addBumpMapping(
                QString const& diffuse_color_map_path,
                QString const& normal_map_path);
        void addBumpMapping(
                std::string const& diffuse_color_map_path,
                std::string const& normal_map_path);
        void removeBumpMapping();

        QVector3D getTranslation() const;
        void setTranslation(QVector3D const& v);
        void setRotation(QQuaternion const& q);

        void setSeperateAxes(bool val = true) {
            show_seperate_axes = val;
            emit propertyChanged("showSeperateAxes");
            updateModel(total_size);
        }
        bool isSeperateAxes() const {
            return show_seperate_axes;
        }
        void setResolution(double res) {
            resolution = res;
            emit propertyChanged("resolution");
            updateModel(total_size);
        }
        double getResolution() const {
            return resolution;
        }

    private:
        base::Vector3d color;
        double total_size;
        double main_size;
        double text_size;

        osg::Vec3 translation;
        osg::Quat rotation;

        enum BODY_TYPES
        { BODY_NONE, BODY_SIMPLE, BODY_SPHERE, BODY_CUSTOM_MODEL };

        BODY_TYPES body_type;
	    osg::ref_ptr<osg::Node>  body_model;
        osg::ref_ptr<osg::Group> createSimpleBody(double size);
	    osg::ref_ptr<osg::Group> createSimpleSphere(double size);

        osg::ref_ptr<osg::Image> image;
        osg::ref_ptr<osg::Texture2D> texture;
        bool texture_dirty;
        void updateTexture();

        osg::ref_ptr<osg::Image> diffuse_image;
        osg::ref_ptr<osg::Image> normal_image;
        osg::ref_ptr<osg::Texture2D> diffuse_texture;
        osg::ref_ptr<osg::Texture2D> normal_texture;
        osg::ref_ptr<osgFX::BumpMapping> bump_mapping;
        bool bump_mapping_dirty;
        void updateBumpMapping();

        osg::ref_ptr<osg::PositionAttitudeTransform> linear_vel_transform, angular_vel_transform;
        osg::ref_ptr<osg::PositionAttitudeTransform> linear_acc_transform, angular_acc_transform;
        osg::ref_ptr<WrenchModel> wrench_model;

        bool forcePositionDisplay;
        bool forceOrientationDisplay;

        bool showPose;
        bool showVelocity;
        bool showAcceleration;
        bool showWrench;
        bool show_seperate_axes;
        double resolution;

        QString model_path;

        std::shared_ptr<osgviz::PrimitivesFactory> primitivesfactory;

};

}
#endif // ROBOT_H
