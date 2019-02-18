#include "SonarVisualization.hpp"
#include <osg/Geode>
#include <base/Eigen.hpp>

namespace vizkit3d
{

SonarVisualization::SonarVisualization():
    fan_opening(0),
    new_sonar_scan(false),
    full_scan(false)
{}

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

double SonarVisualization::angleDifference(double a,double b)
{
    double dif = a - b;
    if(dif < -M_PI)
        dif += 2*M_PI;
    if(dif > M_PI)
        dif -= 2*M_PI;
    return dif;
}

/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void SonarVisualization::updateDataIntern(const base::samples::Sonar& data)
{
    new_sonar_scan = true;
    base::samples::Sonar sample = data;
    if(sample.bearings[0].rad < 0)
        sample.bearings[0].rad += 2*M_PI;

    if(this->data.size() == 0)
    {
        this->data.push_back(sample);
        return;
    }
    
    if(sample.beam_count > 1 || !full_scan)
    {
        this->data.front() = sample;
        return;
    }

    double dist_front = angleDifference(sample.bearings[0].rad , this->data.front().bearings[0].rad);
    double dist_back = angleDifference(sample.bearings[0].rad , this->data.back().bearings[0].rad);
    
    this->data.push_back(sample);
    
    while(abs(dist_front) < abs(dist_back))
    {
        this->data.pop_front();
        dist_front = angleDifference(sample.bearings[0].rad , this->data.front().bearings[0].rad);
    }
}


void SonarVisualization::clearVisualization()
{
    data.clear();
}


void SonarVisualization::showFullScan(bool full_scan)
{
    { 
        boost::mutex::scoped_lock lockit(this->updateMutex);
        this->full_scan = full_scan; 
        data.clear();
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

    int count = 0;
    for(auto it = data.begin(); it != data.end(); it++)
    {
        if(it->beam_count <= 0)
            continue;

        double bin_length = it->bin_duration.toSeconds() * it->speed_of_sound;
        
        for(int i = 0; i < it->beam_count; i++)
        {
            double fan_opening = it->beam_width.getRad();

            osg::Quat left_beam_limit;
            left_beam_limit.makeRotate(it->bearings[i].rad - fan_opening/2.0, 
                osg::Vec3d(0,0,1));
            
            osg::Quat right_beam_limit;
            right_beam_limit.makeRotate(it->bearings[i].rad + fan_opening/2.0, 
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
                pointsOSG->push_back(right_beam_limit*currentPoint);
                pointsOSG->push_back(right_beam_limit*nextPoint);
                pointsOSG->push_back(left_beam_limit*currentPoint);
                //Second Triangle                
                pointsOSG->push_back(left_beam_limit*currentPoint);
                pointsOSG->push_back(left_beam_limit*nextPoint);
                pointsOSG->push_back(right_beam_limit*nextPoint);
            }
            count++;
        }
    }
    drawArrays->setCount(pointsOSG->size());
    pointGeom->setVertexArray(pointsOSG);
    pointGeom->setColorArray(color);
    
}

}
