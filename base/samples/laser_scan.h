#ifndef BASE_SAMPLES_LASER_H__
#define BASE_SAMPLES_LASER_H__

#ifndef __orogen
#include <vector>
#include <boost/cstdint.hpp>
#include <Eigen/Geometry>
#include <stdexcept>
#endif

#include <base/time.h>

namespace base { namespace samples {
    struct LaserScan {
#ifndef __orogen
        typedef boost::uint32_t uint32_t;
#endif

        /** The timestamp of this reading. The timestamp is the time at which the
         * laser passed the zero step (i.e. the step at the back of the device
         */
        Time time;

        /** The angle at which the range readings start. Zero is at the front of
         * the device and turns counter-clockwise. 
	 * This value is in radians
         */
        double start_angle;

        /** Angle difference between two scan point in radians;
         */
        double angular_resolution;

        /** The rotation speed of the laserbeam in radians/seconds
         */
        double speed;

        /** The ranges themselves: the distance to obstacles in millimeters
         */
        std::vector<uint32_t> ranges;

	/** minimal valid range returned by laserscanner */
	uint32_t minRange;
	
	/** maximal valid range returned by laserscanner */
	uint32_t maxRange;
	
        /** The remission value from the laserscan.
	 * This value is not normalised and depends on various factors, like distance, 
	 * angle of incidence and reflectivity of object.
         */
        std::vector<float> remission;

#ifndef __orogen
        LaserScan()
            : start_angle(0), angular_resolution(0), speed(0) {}
            
        bool isValidBeam(const unsigned int i) const {
	    if(i > ranges.size())
		throw std::out_of_range("Invalid beam index given");

	    uint32_t range = ranges[i];
	    if(range > minRange && range < maxRange)
		return true;
	    
	    return false;
	}
            
        bool getPointFromScanBeam(const unsigned int i, Eigen::Vector3d &point) const
	{
	    if(!isValidBeam(i))
		return false;
	    
	    //get a vector with the right length
	    point = Eigen::Vector3d(0.0 , ranges[i] / 1000.0, 0.0);
	    //rotate
	    point = Eigen::Quaterniond(Eigen::AngleAxisd(start_angle + i * angular_resolution, Eigen::Vector3d::UnitZ())) * point;
	    
	    return true;
	}
	
	std::vector<Eigen::Vector3d> convertScanToPointCloud(const Eigen::Transform3d& transform) const
	{
	    std::vector<Eigen::Vector3d> pointCloud;
	    
	    for(unsigned int i = 0; i < ranges.size(); i++) {
		Eigen::Vector3d point;
		if(getPointFromScanBeam(i, point)) {
		    point = transform * point;
		    pointCloud.push_back(point);
		}
	    }
	    
	    return pointCloud;
	}
#endif
    };
}} // namespaces

#endif
