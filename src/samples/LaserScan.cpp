#include "LaserScan.hpp"

#include <stdexcept>

namespace base { namespace samples {

bool LaserScan::isValidBeam(const unsigned int i) const
{
    if(i > ranges.size())
        throw std::out_of_range("Invalid beam index given");
    return isRangeValid(ranges[i]);
}

void LaserScan::reset()
{
    speed = 0.0;
    start_angle = 0.0;
    minRange = 0;
    maxRange = 0;
    ranges.clear();
    remission.clear();
}

bool LaserScan::isRangeValid(LaserScan::uint32_t range) const
{
    if(range >= minRange && range <= maxRange && range >= END_LASER_RANGE_ERRORS)
        return true;
    return false;
}

bool LaserScan::getPointFromScanBeamXForward(const unsigned int i, Eigen::Vector3d& point) const
{
    if(!isValidBeam(i))
        return false;
    
    //get a vector with the right length
    point = Eigen::Vector3d(ranges[i] / 1000.0, 0.0, 0.0);
    //rotate
    point = Eigen::Quaterniond(Eigen::AngleAxisd(start_angle + i * angular_resolution, Eigen::Vector3d::UnitZ())) * point;
    
    return true;
}

bool LaserScan::getPointFromScanBeam(const unsigned int i, Eigen::Vector3d& point) const
{
    if(!isValidBeam(i))
        return false;
    
    //get a vector with the right length
    point = Eigen::Vector3d(0.0 , ranges[i] / 1000.0, 0.0);
    //rotate
    point = Eigen::Quaterniond(Eigen::AngleAxisd(start_angle + i * angular_resolution, Eigen::Vector3d::UnitZ())) * point;
    
    return true;
}

std::vector< Eigen::Vector3d > LaserScan::convertScanToPointCloud(const Eigen::Affine3d& transform) const
{
    std::vector<Eigen::Vector3d> pointCloud;
    pointCloud.reserve(ranges.size());
    
    for(unsigned int i = 0; i < ranges.size(); i++) {
        Eigen::Vector3d point;
        if(getPointFromScanBeam(i, point)) {
            point = transform * point;
            pointCloud.push_back(point);
        }
    }
    
    return pointCloud;
}

}} //end namespace base::samples