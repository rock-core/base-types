#ifndef __GPS_READINGS__
#define __GPS_READINGS__

#include <dfki/time.h>
#include <dfki/linear_algebra.h>

namespace DFKI {
    struct PositionReading {
	/** Timestamp of the position data */
	Time stamp;

	/** Position in ENU frame of reference */
	DFKI::Vector3 position;

	/** Position error (probably as 1sigma?) */
	DFKI::Vector3 error;
    };
}

#endif
