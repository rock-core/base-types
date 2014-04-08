#include "SonarBeamVisualization.hpp"
#include <osg/Geode>
#include <base/Eigen.hpp>

namespace vizkit3d
{

SonarBeamVisualization::SonarBeamVisualization()
{
    bodyState.invalidate();
    newSonarScan = false;
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
    double curStep = fmodf(fabs(lastBeam.bearing.rad - data.bearing.rad),M_PI*2.0);
    double curNumSteps = (2.0*M_PI)/curStep;
    if(curStep < 1){
        if(curNumSteps != numSteps){
            lastStepsize = curStep;
            numSteps = curNumSteps;
            this->data.resize(numSteps+1);
        }
        //Make sure we got valid data, and are able to calculate them
        if(this->data.size() > 40){
            double currentPos = ((data.bearing.rad+M_PI)/(2.0*M_PI))*numSteps;
            BeamHelper bm;
            bm.beam = data;
            bm.orientation = bodyState;
            this->data[currentPos] = bm;
            newSonarScan = true;
        }
    }
    lastBeam = data;
}

void SonarBeamVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    bodyState = data;
}

void SonarBeamVisualization::updateMainNode(osg::Node* node)
{
    if (newSonarScan)
    {
        newSonarScan = false;

        //Trivial check if we have data
        if(data.size() == 0) return;

        //Cleaning up internal structures
        pointsOSG->clear();
        color->clear();
       
        //Pre-initialize old heading to prevent artefacts
        osg::Quat lastBeamOrientation;
        lastBeamOrientation.makeRotate(data[0].orientation.getYaw() + data[0].beam.bearing.rad+M_PI - lastStepsize/2.0, osg::Vec3d(0,0,1));

        for(std::vector<BeamHelper>::const_iterator it = data.begin(); it != data.end(); it++)
        {
            osg::Quat currrentBeamOrientation;
            currrentBeamOrientation.makeRotate(it->orientation.getYaw() + it->beam.bearing.rad+M_PI - lastStepsize/2.0, osg::Vec3d(0,0,1));
            for(size_t i =0;i<it->beam.beam.size();i++)
            {
                osg::Vec3d currentPoint(it->beam.getSpatialResolution() * i, 0,0);
                osg::Vec3d nextPoint(it->beam.getSpatialResolution() * (i+1),0,0);

                double v = it->beam.beam[i]/255.0;
                if(v!= 0.0){
                    color->push_back(osg::Vec4f(v,0,0,1.0));
                    color->push_back(osg::Vec4f(v,0,0,1.0));
                    color->push_back(osg::Vec4f(v,0,0,1.0));
                    color->push_back(osg::Vec4f(v,0,0,1.0));
                    color->push_back(osg::Vec4f(v,0,0,1.0));
                    color->push_back(osg::Vec4f(v,0,0,1.0));
                }else{
                    color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
                    color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
                    color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
                    color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
                    color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
                    color->push_back(osg::Vec4f(0.0,0.0,0.0,0.0));
                }
                
                //First triangle
                pointsOSG->push_back(currrentBeamOrientation*currentPoint);
                pointsOSG->push_back(currrentBeamOrientation*nextPoint);
                pointsOSG->push_back(lastBeamOrientation*currentPoint);
                //Second Triangle                
                pointsOSG->push_back(lastBeamOrientation*currentPoint);
                pointsOSG->push_back(lastBeamOrientation*nextPoint);
                pointsOSG->push_back(currrentBeamOrientation*nextPoint);
            }
            lastBeamOrientation = currrentBeamOrientation;
        }
            
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        pointGeom->setColorArray(color);
    }
}

}
