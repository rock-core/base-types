#ifndef __GPS_READINGS__
#define __GPS_READINGS__

#include <base/time.h>
#include <base/linear_algebra.h>

namespace base {
    struct PositionReading {
	/** Timestamp of the position data */
	Time stamp;

	/** Position in ENU frame of reference */
	Vector3 position;

	/** Position error (probably as 1sigma?) */
	Vector3 error;
    };
}

#endif
