#include "SonarVisualization.hpp"
#include <osg/Geode>
#include <base/Eigen.hpp>

namespace vizkit3d
{

SonarVisualization::SonarVisualization():
    new_sonar_scan(false)
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


/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void SonarVisualization::updateDataIntern(const base::samples::Sonar& data)
{
    new_sonar_scan = true;
    if(data.beam_count <=0 || data.bins.size() <=0)
        throw std::runtime_error("Invalid sonar data");
    last_sonar = data;
}


void SonarVisualization::updateMainNode(osg::Node* node)
{
    if (!new_sonar_scan)
        return;
    new_sonar_scan = false;

    //Cleaning up internal structures
    pointsOSG->clear();
    color->clear();

    double bin_length = last_sonar.bin_duration.toSeconds() * last_sonar.speed_of_sound;
        
    for(size_t i = 0; i < last_sonar.beam_count; i++)
    {
        double fan_opening = last_sonar.beam_width.getRad();

        osg::Quat left_beam_limit;
        left_beam_limit.makeRotate(last_sonar.bearings[i].rad - fan_opening/2.0, 
            osg::Vec3d(0,0,1));
        
        osg::Quat right_beam_limit;
        right_beam_limit.makeRotate(last_sonar.bearings[i].rad + fan_opening/2.0, 
            osg::Vec3d(0,0,1));
        
        int fan_segment = last_sonar.bin_count/last_sonar.beam_count;

        for(size_t j = i*fan_segment; j < (i+1)*fan_segment; j++)
        {
            osg::Vec3d currentPoint(bin_length * (j - i*fan_segment), 0,0);
            osg::Vec3d nextPoint(bin_length * ((j+1) - i*fan_segment) ,0,0);

            double v = last_sonar.bins[j];
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
    }
    drawArrays->setCount(pointsOSG->size());
    pointGeom->setVertexArray(pointsOSG);
    pointGeom->setColorArray(color);
    
}

}
