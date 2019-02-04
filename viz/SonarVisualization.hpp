#ifndef SonarVisualization_H
#define SonarVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/Sonar.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <osg/Node>
#include <osg/Geometry>

namespace vizkit3d
{


struct SonarHelper{
    base::samples::Sonar sonar_sample;
    base::samples::RigidBodyState orientation;
};

/**
 * Vizkit plugin to visualize sonar data.
 * 
 * If the class gets updated with a body state the sonar 
 * data is absolute, otherwise relative.
 */
class SonarVisualization : public vizkit3d::Vizkit3DPlugin< base::samples::Sonar >,
                               public vizkit3d::VizPluginAddType< base::samples::RigidBodyState >
{    
    Q_OBJECT
    Q_PROPERTY(double MotorStep READ getMotorStep WRITE setMotorStep)
    Q_PROPERTY(bool FullScan READ isFullScan WRITE setFullScan)
    
    public:
        SonarVisualization();
    
        Q_INVOKABLE void updateSonar( const base::samples::Sonar& sample )
        { return updateData(sample); }
        Q_INVOKABLE void updateOrientation( const base::samples::RigidBodyState& orientation )
        { return updateData(orientation); }
    
    public slots:
        void setMotorStep(double step);
        double getMotorStep();
        void setFullScan(bool full_scan);
        bool isFullScan();
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const base::samples::Sonar& data );
        void updateDataIntern ( const base::samples::RigidBodyState& data );
        
    private:
        base::samples::Sonar last_sample;
        base::samples::RigidBodyState last_orientation;
        double motor_step;
        double num_steps;
        std::vector<SonarHelper> data;

        bool new_sonar_scan;
        bool full_scan;

        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
};

}
#endif
