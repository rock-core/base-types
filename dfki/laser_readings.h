#ifndef DFKI_LASER_READINGS_H__
#define DFKI_LASER_READINGS_H__

#ifndef __orogen
#include <vector>
#endif

#include <dfki/base_types.h>

namespace DFKI {
    struct LaserReadings {
        /** The timestamp of this reading. The timestamp is the time at which the
         * laser passed the zero step (i.e. the step at the back of the device
         */
        Timestamp stamp;

        /** The step at which the range readings start. Step zero is at the back of
         * the device and turns counter-clockwise. There is \c resolution steps per
         * turn 
         *
         * There are (max - min + 1) steps
         */
        uint32_t min;

        /** How much steps there is per turn
         */
        uint32_t resolution;

        /** The rotation speed in microseconds per step
         */
        uint32_t speed;

        /** The ranges themselves
         *
         * There are (max - min + 1) ranges in this array
         */
        std::vector<uint32_t> ranges;
    };
}

#endif
