#ifndef BASE_TYPES_LASER_READINGS_H__
#define BASE_TYPES_LASER_READINGS_H__

#ifndef __orogen
#include <vector>
#include <boost/cstdint.hpp>
#endif

#include <base/base_types.h>

namespace base {
    struct LaserReadings {
#ifndef __orogen
        typedef boost::uint32_t uint32_t;
#endif

        /** The timestamp of this reading. The timestamp is the time at which the
         * laser passed the zero step (i.e. the step at the back of the device
         */
        Time stamp;

        /** The step at which the range readings start. Step zero is at the back of
         * the device and turns counter-clockwise. There is \c resolution steps per
         * turn 
         */
        uint32_t min;

        /** How much steps there is per turn
         */
        uint32_t resolution;

        /** The rotation speed in microseconds per step
         */
        uint32_t speed;

        /** The ranges themselves: the distance to obstacles in millimeters
         */
        std::vector<uint32_t> ranges;

#ifndef __orogen
        LaserReadings()
            : min(0), resolution(0), speed(0) {}
#endif
    };
}

#endif
