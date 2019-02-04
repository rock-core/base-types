#include "SonarVisualization.hpp"
#include <osg/Geode>
#include <base/Eigen.hpp>

namespace vizkit3d
{

SonarVisualization::SonarVisualization():
    motor_step(0),
    num_steps(0),
    new_sonar_scan(false),
    full_scan(false)
{
    last_orientation.invalidate();
    data.resize(1);
}

osg::ref_ptr< osg::Node > SonarVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    color = new osg::Vec4Array;
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
void SonarVisualization::updateDataIntern(const base::samples::Sonar& data)
{
    if(motor_step == 0)
        return;
    double current_pos = 0;
    if(full_scan)
        current_pos = ((data.bearings[0].rad+M_PI)/(2.0*M_PI))*num_steps;

    SonarHelper bm;
    bm.sonar_sample = data;
    bm.orientation = last_orientation;
    this->data[current_pos] = bm;
    new_sonar_scan = true;
}

void SonarVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    last_orientation = data;
}

void SonarVisualization::setMotorStep(double motor_step)
{
    { 
        boost::mutex::scoped_lock lockit(this->updateMutex);
        if (motor_step == 0)
            return; //Exception?
        this->motor_step = motor_step;
        this->num_steps =  (2.0*M_PI)/motor_step;
        if(full_scan)
            this->data.resize(num_steps+1);
    }
    emit propertyChanged("Motor Step");
    setDirty();
} 

double SonarVisualization::getMotorStep()
{
    return motor_step;
}

void SonarVisualization::setFullScan(bool full_scan)
{
    { 
        //boost::mutex::scoped_lock lockit(this->updateMutex);
        this->full_scan = full_scan; 
        data.clear();
        if(!full_scan)
        {
            data.resize(1);    
        }
        else
        {
            if(motor_step != 0)
            {
                this->data.resize(num_steps+1);
            }
        }
        
    }
    emit propertyChanged("Full Scan");
    setDirty();
}

bool SonarVisualization::isFullScan()
{
    return full_scan;
}

void SonarVisualization::updateMainNode(osg::Node* node)
{
    if (new_sonar_scan)
    {
        new_sonar_scan = false;

        //Trivial check if we have data
        if(data.size() == 0) return;

        //Cleaning up internal structures
        pointsOSG->clear();
        color->clear();

        for(auto it = data.begin(); it != data.end(); it++)
        {
            if(it->sonar_sample.beam_count <= 0)
                continue;
            //Pre-initialize old heading to prevent artefacts
            osg::Quat lastBeamOrientation;
            lastBeamOrientation.makeRotate(it->orientation.getYaw() + 
                it->sonar_sample.bearings[0].rad - motor_step/2.0, osg::Vec3d(0,0,1));
            
            osg::Quat currrentBeamOrientation;
            currrentBeamOrientation.makeRotate(it->orientation.getYaw() + 
                it->sonar_sample.bearings[0].rad + motor_step/2.0, osg::Vec3d(0,0,1));
            
            for(size_t i =0; i < it->sonar_sample.bin_count; i++)
            {
                double bin_length = it->sonar_sample.bin_duration.toSeconds()*
                    it->sonar_sample.getSpeedOfSoundInWater();
                osg::Vec3d currentPoint(bin_length * i, 0,0);
                osg::Vec3d nextPoint(bin_length * (i+1),0,0);

                double v = it->sonar_sample.bins[i];
                if(v!= 0.0){
                    color->push_back(osg::Vec4f(v, 0, 0, 1.0));
                    color->push_back(osg::Vec4f(v, 0, 0, 1.0));
                    color->push_back(osg::Vec4f(v, 0, 0, 1.0));
                    color->push_back(osg::Vec4f(v, 0, 0, 1.0));
                    color->push_back(osg::Vec4f(v, 0, 0, 1.0));
                    color->push_back(osg::Vec4f(v, 0, 0, 1.0));
                }else{
                    color->push_back(osg::Vec4f(0, 0, 1.0, .5));
                    color->push_back(osg::Vec4f(0, 0, 1.0, .5));
                    color->push_back(osg::Vec4f(0, 0, 1.0, .5));
                    color->push_back(osg::Vec4f(0, 0, 1.0, .5));
                    color->push_back(osg::Vec4f(0, 0, 1.0, .5));
                    color->push_back(osg::Vec4f(0, 0, 1.0, .5));
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
        }
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        pointGeom->setColorArray(color);
    }
}

}
