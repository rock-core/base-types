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
    };
}

#endif
