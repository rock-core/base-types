#ifndef SonarVisualization_H
#define SonarVisualization_H

#include <list>

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/Sonar.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <osg/Node>
#include <osg/Geometry>

namespace vizkit3d
{

/**
 * Vizkit plugin to visualize sonar data.
 * 
 * If the class gets updated with a body state the sonar 
 * data is absolute, otherwise relative.
 */
class SonarVisualization : public vizkit3d::Vizkit3DPlugin< base::samples::Sonar >
{    
    Q_OBJECT
    
    public:
        SonarVisualization();
    
        Q_INVOKABLE void updateSonar( const base::samples::Sonar& sample )
        { return updateData(sample); }
    
    public slots:
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const base::samples::Sonar& data );
 
    private:

        bool new_sonar_scan;
        base::samples::Sonar last_sonar;

        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
};

}
#endif
