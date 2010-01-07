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

        /** The remission value from the laserscan.
	 * This value is not normalised and depends on various factors, like distance, 
	 * angle of incidence and reflectivity of object.
         */
        std::vector<float> remission;

#ifndef __orogen
        LaserScan()
            : min(0), resolution(0), speed(0) {}
#endif
    };
}} // namespaces

#endif
