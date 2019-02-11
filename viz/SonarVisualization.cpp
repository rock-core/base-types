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
    if(motor_step == 0 && data.beam_count == 1)
        return;
    
    new_sonar_scan = true;
    
    int current_pos = 0;
    if(data.beam_count > 1)
    {
        this->data[current_pos] = data;
        return;
    }
    else
    {
        if(full_scan)
        {
            current_pos = std::round((data.bearings[0].rad + M_PI)/motor_step);
        }

        this->data[current_pos] = data;
        new_sonar_scan = true;
    }
}


void SonarVisualization::clearVisualization()
{
    data.clear();
}


void SonarVisualization::setMotorStep(double motor_step)
{
    if (motor_step == 0)
        return; 
    { 
        boost::mutex::scoped_lock lockit(this->updateMutex);
        this->motor_step = motor_step;
        this->num_steps =  std::round((2.0*M_PI)/motor_step);
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

void SonarVisualization::showFullScan(bool full_scan)
{
    { 
        boost::mutex::scoped_lock lockit(this->updateMutex);
        this->full_scan = full_scan; 
        data.clear();
        if(!full_scan)
        {
            data.resize(1);    
        }
        else if(motor_step != 0)
        {
            this->data.resize(num_steps+1);
        }
    }
    emit propertyChanged("FullScan");
    setDirty();
}

bool SonarVisualization::isFullScan()
{
    return full_scan;
}

void SonarVisualization::updateMainNode(osg::Node* node)
{
    if (!new_sonar_scan)
        return;
    new_sonar_scan = false;

    //Cleaning up internal structures
    pointsOSG->clear();
    color->clear();

    for(auto it = data.begin(); it != data.end(); it++)
    {
        if(it->beam_count <= 0)
            continue;

        double fan_opening;

        if(it->beam_count > 1)
            fan_opening = it->bearings[1].rad - it->bearings[0].rad;
        else
            fan_opening = motor_step;

        double bin_length = it->bin_duration.toSeconds() * it->speed_of_sound;
        
        for(int i = 0; i < it->beam_count; i++)
        {
            osg::Quat lastBeamOrientation;
            lastBeamOrientation.makeRotate(it->bearings[i].rad - fan_opening/2.0, 
                osg::Vec3d(0,0,1));
            
            osg::Quat currrentBeamOrientation;
            currrentBeamOrientation.makeRotate(it->bearings[i].rad + fan_opening/2.0, 
                osg::Vec3d(0,0,1));
            
            int fan_segment = it->bin_count/it->beam_count;

            for(size_t j = i*fan_segment; j < (i+1)*fan_segment; j++)
            {
                osg::Vec3d currentPoint(bin_length * (j - i*fan_segment), 0,0);
                osg::Vec3d nextPoint(bin_length * ((j+1) - i*fan_segment) ,0,0);

                double v = it->bins[j];
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
    }
    drawArrays->setCount(pointsOSG->size());
    pointGeom->setVertexArray(pointsOSG);
    pointGeom->setColorArray(color);
    
}

}
