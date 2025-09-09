#ifndef teleop_mapping_WrenchVisualization_H
#define teleop_mapping_WrenchVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/samples/Wrench.hpp>
#include <osgText/Text>
#include <osg/Switch>
#include <osg/AnimationPath>
#include "WrenchModel.hpp"

namespace vizkit3d
{
    class WrenchVisualization
        : public vizkit3d::Vizkit3DPlugin<base::samples::Wrench>
        , public vizkit3d::VizPluginAddType<base::Wrench>
    {
    Q_OBJECT
    Q_PROPERTY(double textSize READ getTextSize WRITE setTextSize)
    Q_PROPERTY(bool showForceSeperateAxes READ isSeperateAxesForce WRITE setSeperateAxesForce)
    Q_PROPERTY(bool showTorqueSeperateAxes READ isSeperateAxesTorque WRITE setSeperateAxesTorque)
    Q_PROPERTY(double resolution READ getResolution WRITE setResolution)
    public:
        WrenchVisualization();
        ~WrenchVisualization();

        Q_INVOKABLE void updateData(base::samples::Wrench const &sample)
        {vizkit3d::Vizkit3DPlugin<base::samples::Wrench>::updateData(sample);}
        
        Q_INVOKABLE void updateData(base::Wrench const &sample)
        {vizkit3d::Vizkit3DPlugin<base::samples::Wrench>::updateData(sample);}

        Q_INVOKABLE void setForce(const base::Vector3d& f) {
            state.force = f;
        }

        Q_INVOKABLE void setTorque(const base::Vector3d& t) {
            state.torque = t;
        }

    public slots: 
        /** Sets the text size relative to the size of the complete object.
         * If text size is positive, the name of the source frame is rendered in the visualization.
         * The default is 0.0
         */
        void setTextSize(double size);
        double getTextSize() const;

        void setSeperateAxesForce(bool val = true) {
            show_seperate_axes_force = val;
            emit propertyChanged("showForceSeperateAxes");
            wrench_model->seperateAxesForce(val);
            setDirty();
        }
        bool isSeperateAxesForce() const {
            return show_seperate_axes_force;
        }
        void setSeperateAxesTorque(bool val = true) {
            show_seperate_axes_torque = val;
            emit propertyChanged("showTorqueSeperateAxes");
            wrench_model->seperateAxesTorque(val);            
            setDirty();
        }
        bool isSeperateAxesTorque() const {
            return show_seperate_axes_torque;
        }
        void setResolution(double res) {
            resolution = res;
            emit propertyChanged("resolution");
            wrench_model->setResolution(res);
            setDirty();
        }
        double getResolution() const {
            return resolution;
        }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern( const base::Wrench& wrench );
        virtual void updateDataIntern( const base::samples::Wrench& wrench );
  
    private:
        base::samples::Wrench state;
        osg::ref_ptr<WrenchModel> wrench_model;
        osg::ref_ptr<osgText::Text> text;
        double text_size;
        bool show_seperate_axes_force;
        bool show_seperate_axes_torque;
        double resolution;

    // public slots: 
    //     void setForce(const base::Vector3d& f) {
    //         state.force = f;
    //     }

    //     void setTorque(const base::Vector3d& t) {
    //         state.torque = t;
    //     }

    };
}
#endif
