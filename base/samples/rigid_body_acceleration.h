#ifndef __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH
#define __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH

#ifdef __orogen
#error "this header cannot be used. Use the wrapper version form Acceleration"
#endif

#include <base/pose.h>
#include <base/time.h>

namespace base { namespace samples {
    struct RigidBodyAcceleration 
    {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        base::Time time;

        /** Acceleration in m/s, world fixed frame of reference (East-North-Up) */
        Eigen::Vector3d acceleration;
	/** Covariance matrix of the acceleration
	 */
        Eigen::Matrix3d cov_acceleration;

	void invalidateOrientation() {
	    cov_acceleration = Eigen::Matrix3d::Identity();
	    cov_acceleration *= INFINITY;
	}
	
    };
}}

#endif

