#include "SonarBeamVisualization.hpp"
#include <osg/Geode>
#include <base/Eigen.hpp>

namespace vizkit3d
{

SonarBeamVisualization::SonarBeamVisualization()
{
    bodyState.invalidate();
    newSonarScan = false;
    setKeepOldData(true);
    setMaxOldData(100);
}

osg::ref_ptr< osg::Node > SonarBeamVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    color = new osg::Vec4Array;
    //color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::TRIANGLES, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    return mainNode;
}

/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void SonarBeamVisualization::updateDataIntern(const base::samples::SonarBeam& data)
{
    lastBeam = data;
}

void SonarBeamVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    bodyState = data;
}

void SonarBeamVisualization::updateMainNode(osg::Node* node)
{
    //Cleaning up internal structures
    pointsOSG->clear();
    color->clear();
   
    osg::Quat beamLeft, beamRight;
    beamLeft.makeRotate(lastBeam.bearing.rad - lastBeam.beamwidth_horizontal / 2, osg::Vec3d(0,0,1));
    beamRight.makeRotate(lastBeam.bearing.rad + lastBeam.beamwidth_horizontal / 2, osg::Vec3d(0,0,1));

    for(size_t i =0;i < lastBeam.beam.size();i++)
    {
        osg::Vec3d currentPoint(lastBeam.getSpatialResolution() * i, 0,0);
        osg::Vec3d nextPoint(lastBeam.getSpatialResolution() * (i+1),0,0);

        double v = lastBeam.beam[i]/255.0;
        if(v!= 0.0){
            color->push_back(osg::Vec4f(v,v/4,v/4,1.0));
            color->push_back(osg::Vec4f(v,v/4,v/4,1.0));
            color->push_back(osg::Vec4f(v,v/4,v/4,1.0));
            color->push_back(osg::Vec4f(v,v/4,v/4,1.0));
            color->push_back(osg::Vec4f(v,v/4,v/4,1.0));
            color->push_back(osg::Vec4f(v,v/4,v/4,1.0));
        }else{
            color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
            color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
            color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
            color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
            color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
            color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
        }
        
        //First triangle
        pointsOSG->push_back(beamLeft*currentPoint);
        pointsOSG->push_back(beamLeft*nextPoint);
        pointsOSG->push_back(beamRight*currentPoint);
        //Second Triangle                
        pointsOSG->push_back(beamRight*currentPoint);
        pointsOSG->push_back(beamRight*nextPoint);
        pointsOSG->push_back(beamLeft*nextPoint);
    }
            
    drawArrays->setCount(pointsOSG->size());
    pointGeom->setVertexArray(pointsOSG);
    pointGeom->setColorArray(color);
}

}
