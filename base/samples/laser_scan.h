#ifndef BASE_SAMPLES_LASER_H__
#define BASE_SAMPLES_LASER_H__

#ifdef __GCCXML__
#define EIGEN_DONT_VECTORIZE
#endif

#ifndef __orogen
#include <vector>
#include <boost/cstdint.hpp>
#include <Eigen/Geometry>
#include <stdexcept>
#endif

#ifdef __GNUC__
    #define DEPRICATED __attribute__ ((deprecated))
#else
    #define DEPRICATED
#endif 

#include <base/time.h>

namespace base { namespace samples {
    /** Special values for the ranges. If a range has one of these values, then
    * it is not valid and the value declares what is going on */
    enum LASER_RANGE_ERRORS {
        TOO_FAR            = 1, // too far
        TOO_NEAR           = 2,
        MEASUREMENT_ERROR  = 3,
        OTHER_RANGE_ERRORS = 4,
        MAX_RANGE_ERROR    = 5  
    };

    struct LaserScan {
#ifndef __orogen
        typedef boost::uint32_t uint32_t;
#endif

        /** The timestamp of this reading. The timestamp is the time at which the
         * laser passed the zero step (i.e. the step at the back of the device,
         * which is distinct from the measurement 0)
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
        
        //resets the sample
        void reset()
        {
          speed = 0.0;
          start_angle = 0.0;
          minRange = 0;
          maxRange = 0;
          ranges.clear();
          remission.clear();
        }

        /** converts the laser scan into a point cloud according to the given transformation matrix,
         *  the start_angle and the angular_resolution. If the transformation matrix is set to 
         *  identity the laser scan is converted into the coordinate system of the sensor (x-axis = forward,
         *  y-axis = to the left, z-axis = upwards)
         */
	void convertScanToPointCloud(std::vector<Eigen::Vector3d> &points,
                                     const Eigen::Affine3d& transform = Eigen::Affine3d::Identity())const
        {
            float angle = start_angle;
            double val;

	    points.resize(ranges.size());
            std::vector<Eigen::Vector3d>::iterator point_iter = points.begin();
            std::vector<uint32_t>::const_iterator range_iter = ranges.begin();
	    for(;range_iter != ranges.end();++point_iter,++range_iter) 
            {
                //check if point is valid
	        if(*range_iter < minRange || *range_iter > maxRange)
		    continue;

                //convert from millimeters to meter
                val = 0.001*(*range_iter);
                //rotate because of the scan line angle
                //this is a very special rotation (y = 0, z = 0)
                point_iter->x() = val*cos(angle);
                point_iter->y() = val*sin(angle);
                point_iter->z() = 0;
                //transform
		*point_iter = transform * (*point_iter);
                angle += angular_resolution;
	    }
	}
            
        /** \deprecated
         * returns the points in a wrong coordinate system
         */
        bool getPointFromScanBeam(const unsigned int i, Eigen::Vector3d &point) const DEPRICATED
	{
	    if(!isValidBeam(i))
		return false;
	    
	    //get a vector with the right length
	    point = Eigen::Vector3d(0.0 , ranges[i] / 1000.0, 0.0);
	    //rotate
	    point = Eigen::Quaterniond(Eigen::AngleAxisd(start_angle + i * angular_resolution, Eigen::Vector3d::UnitZ())) * point;
	    
	    return true;
	}

        /** \deprecated - 
         * returns the points in a wrong coordinate system
         */
	std::vector<Eigen::Vector3d> convertScanToPointCloud(const Eigen::Affine3d& transform) const DEPRICATED
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
