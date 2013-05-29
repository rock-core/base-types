#ifndef __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH
#define __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH

#include <base/Eigen.hpp>
#include <base/Time.hpp>

namespace base { namespace samples {
    struct RigidBodyAcceleration 
    {
        base::Time time;

        /** Acceleration in m/s, world fixed frame of reference (East-North-Up) */
        base::Vector3d acceleration;
	/** Covariance matrix of the acceleration
	 */
        base::Matrix3d cov_acceleration;

	void invalidateOrientation() {
	    cov_acceleration = Eigen::Matrix3d::Identity();
	    cov_acceleration *= INFINITY;
	}
	
    };
}}

#endif

