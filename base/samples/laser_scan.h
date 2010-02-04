#ifndef BASE_SAMPLES_LASER_H__
#define BASE_SAMPLES_LASER_H__

#ifndef __orogen
#include <vector>
#include <boost/cstdint.hpp>
#endif

#include <base/time.h>

namespace base { namespace samples {
    struct LaserScan {
#ifndef __orogen
        typedef boost::uint32_t uint32_t;
#endif

        /** The timestamp of this reading. The timestamp is the time at which the
         * laser passed the zero step (i.e. the step at the back of the device
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

        /** The remission value from the laserscan.
	 * This value is not normalised and depends on various factors, like distance, 
	 * angle of incidence and reflectivity of object.
         */
        std::vector<float> remission;

#ifndef __orogen
        LaserScan()
            : start_angle(0), angular_resolution(0), speed(0) {}
#endif
    };
}} // namespaces

#endif
