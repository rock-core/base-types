#ifndef BASE_SAMPLES_LASER_H__
#define BASE_SAMPLES_LASER_H__

#include <vector>
#include <Eigen/Geometry>
#include <base/Float.hpp>
#include <boost/cstdint.hpp>
#include <base/Time.hpp>
#include <base/Deprecated.hpp>

namespace base { namespace samples {
    /** Special values for the ranges. If a range has one of these values, then
    * it is not valid and the value declares what is going on */
    enum LASER_RANGE_ERRORS {
        TOO_FAR            = 1, // too far
        TOO_NEAR           = 2,
        MEASUREMENT_ERROR  = 3,
        OTHER_RANGE_ERRORS = 4,
        MAX_RANGE_ERROR    = 5,
        END_LASER_RANGE_ERRORS
    };

    struct LaserScan {
        typedef boost::uint32_t uint32_t;

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

        LaserScan()
            : start_angle(0), angular_resolution(0), speed(0), minRange(0), maxRange(0) {}
            
        bool isValidBeam(const unsigned int i) const;
        
        //resets the sample
        void reset();

        bool isRangeValid(uint32_t range) const;

        /** converts the laser scan into a point cloud according to the given transformation matrix,
         *  the start_angle and the angular_resolution. If the transformation matrix is set to 
         *  identity the laser scan is converted into the coordinate system of the sensor (x-axis = forward,
         *  y-axis = to the left, z-axis = upwards)
         *  If a scan point is outside of valid range all its coordinates are set to NaN.
         *  Unfortunately invalid scan points can not be skipped because this would invalidate the remission association
         */
        template<typename T>
	void convertScanToPointCloud(std::vector<T> &points,
                                     const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
                                     bool skip_invalid_points = true)const
        {
	    points.clear();
	    
	    //give the vector a hint about the size it might be
	    points.reserve(ranges.size());

	    //this is optimized for runtime
	    //moving the check for skip_invalid_points
	    //out of the loop speeds up the execution 
	    //time by ~25 %
	    if(!skip_invalid_points)
	    {
		for(unsigned int i = 0; i < ranges.size(); i++) {
		    Eigen::Vector3d point;
		    if(getPointFromScanBeamXForward(i, point)) {
			point = transform * point;
			points.push_back(point);
		    } else {
			points.push_back(Eigen::Vector3d(base::unknown<double>(), base::unknown<double>(), base::unknown<double>()));
		    }
		}
	    } else {
		for(unsigned int i = 0; i < ranges.size(); i++) {
		    Eigen::Vector3d point;
		    if(getPointFromScanBeamXForward(i, point)) {
			point = transform * point;
			points.push_back(point);
		    }
		}
	    }
	}
            
        /**
         * Helper function that converts range 'i' to a point.
	 * The origin ot the point will be the laserScanner
         */
        bool getPointFromScanBeamXForward(const unsigned int i, Eigen::Vector3d &point) const;

        /** \deprecated - 
         * returns the points in a wrong coordinate system
         */
        bool getPointFromScanBeam(const unsigned int i, Eigen::Vector3d &point) const;

        /** \deprecated - 
         * returns the points in a wrong coordinate system
         */
	std::vector<Eigen::Vector3d> convertScanToPointCloud(const Eigen::Affine3d& transform) const BASE_TYPES_DEPRECATED;
    };
}} // namespaces

#endif
