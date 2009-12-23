#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__

#include <base/time.h>
#include <base/linear_algebra.h>

namespace base {
    struct SystemState {
	/** Timestamp of the system state */
	Time stamp;

	/** Position in m, 
	 * world fixed frame of reference (East-North-Up) */
	base::Vector3 position;

	/** Orientation of the robot, 
	 * quaternion gives Transformation Body->World */
	base::Quaternion orientation;

	/** Velocity in m/s with respect to world fixed frame, 
	 * in body fixed frame (Right-Front-Up) */
	base::Vector3 velocity;

	/** Angular Velocity in rad/s,
	 * in body fixed frame (Right-Front-Up) */
	base::Vector3 angular_velocity;

	/** Covariance matrix of the position
	 */
	base::Matrix3 cov_position;

	/** Covariance matrix of the orientation as an 
	 * axis/angle manifold in body coordinates
	 */
	base::Matrix3 cov_orientation;

	/** Covariance of the velocity 
	 */
	base::Matrix3 cov_velocity;

	/** Covariance of the angular velocity
	 */
	base::Matrix3 cov_angular_velocity;
    };
}

#endif
