#ifndef __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH
#define __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH

#ifdef __GCCXML__
#define EIGEN_DONT_VECTORIZE
#endif

#ifdef __orogen
#error "this header cannot be used. Use the wrapper version form Acceleration"
#endif

#include <base/eigen.h>
#include <base/time.h>

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

