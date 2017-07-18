#ifndef __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH
#define __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH

#include <base/Eigen.hpp>
#include <base/Time.hpp>

namespace base { namespace samples {
    struct RigidBodyAcceleration 
    {
        base::Time time;

        /** Linear acceleration in m/s2, world fixed frame of reference (East-North-Up) */
        base::Vector3d acceleration;
	/** Covariance matrix of the linear acceleration
	 */
        base::Matrix3d cov_acceleration;

        /** Angular acceleration in rad/s2, world fixed frame of reference (East-North-Up) */
        base::Vector3d angular_acceleration;
	/** Covariance matrix of the angular acceleration
	 */
        base::Matrix3d cov_angular_acceleration;

	void invalidateOrientation();
	
    };
}}

#endif

