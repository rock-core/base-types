#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__

#include <dfki/time.h>
#include <dfki/linear_algebra.h>

namespace DFKI {
    struct SystemState {
	/** Timestamp of the system state */
	Time stamp;

	/** Position in m, 
	 * world fixed frame of reference (East-North-Up) */
	DFKI::Vector3 position;

	/** Orientation of the robot, 
	 * quaternion gives Transformation Body->World */
	DFKI::Quaternion orientation;

	/** Velocity in m/s with respect to world fixed frame, 
	 * in body fixed frame (Right-Front-Up) */
	DFKI::Vector3 velocity;

	/** Angular Velocity in rad/s,
	 * in body fixed frame (Right-Front-Up) */
	DFKI::Vector3 angular_velocity;

	/** Covariance matrix of the position
	 */
	DFKI::Matrix3 cov_position;

	/** Covariance matrix of the orientation as an 
	 * axis/angle manifold in body coordinates
	 */
	DFKI::Matrix3 cov_orientation;

	/** Covariance of the velocity 
	 */
	DFKI::Matrix3 cov_velocity;

	/** Covariance of the angular velocity
	 */
	DFKI::Matrix3 cov_angular_velocity;
    };
}

#endif
