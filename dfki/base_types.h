#ifndef DFKI_BASE_TYPES_H__
#define DFKI_BASE_TYPES_H__

#ifndef __orogen
#include <sys/time.h>
#include <time.h>
#include <stdint.h>
#endif

namespace DFKI {
    struct Time {
        /** The number of seconds */
        int seconds;
        /** The number of microseconds. This is always in [0, 1000000]. */
        int microseconds;

#ifndef __orogen
        Time()
            : seconds(0), microseconds(0) {}

        explicit Time(int seconds, int microseconds = 0)
            : seconds(seconds), microseconds(microseconds) {}

        /** Returns the current time */
        static Time now() {
            timeval t;
            gettimeofday(&t, 0);
            return Time(t.tv_sec, t.tv_usec);
        }

        bool operator < (Time const& ts) const
        { return seconds < ts.seconds || (seconds == ts.seconds && microseconds < ts.microseconds); }
        bool operator > (Time const& ts) const
        { return seconds > ts.seconds || (seconds == ts.seconds && microseconds > ts.microseconds); }
        bool operator == (Time const& ts) const
        { return seconds == ts.seconds && microseconds == ts.microseconds; }
        bool operator != (Time const& ts) const
        { return !(*this == ts); }
        bool operator >= (Time const& ts) const
        { return !(*this < ts); }
        bool operator <= (Time const& ts) const
        { return !(*this > ts); }
        Time operator - (Time const& ts) const
        {
            Time result;
            result.seconds      = seconds - ts.seconds;
            result.microseconds = microseconds - ts.microseconds;
            result.canonize();
            return result;
        }

        Time operator + (Time const& ts) const
        {
            Time result;
            result.seconds      = seconds + ts.seconds;
            result.microseconds = microseconds + ts.microseconds;
            result.canonize();
            return result;
        }

        Time operator / (int divider) const
        {
            Time result;
	    uint64_t timeInMicroSec = this->toMicroseconds();
	    
	    timeInMicroSec /= divider;

            int const UsecPerSec = 1000000;
            int64_t offset = timeInMicroSec / UsecPerSec;
            result.seconds = offset;
            timeInMicroSec -= offset * UsecPerSec;
	    result.microseconds = timeInMicroSec;	  
            return result;
        }

        /** True if this time is zero */
        bool isNull() const { return seconds == 0 && microseconds == 0; }

        /** Converts this time as a timeval object */
        timeval toTimeval() const
        {
            timeval tv = { seconds, microseconds };
            return tv;
        }

        /** Returns this time as a fractional number of seconds */
        double toSeconds() const
        {
            return static_cast<double>(seconds) + static_cast<double>(microseconds) / 1000000.0;
        }

        /** Returns this time as an integer number of milliseconds (thus dropping the microseconds) */
        uint64_t toMilliseconds() const
        {
            return static_cast<uint64_t>(seconds) * 1000 + static_cast<uint64_t>(microseconds) / 1000;
        }

        /** Returns this time as an integer number of microseconds */
        uint64_t toMicroseconds() const
        {
            return static_cast<uint64_t>(seconds) * 1000000 + static_cast<uint64_t>(microseconds);
        }


    private:
        /** This method makes sure that the constraint on microseconds is met
         * (i.e. that microseconds is in [0, 1000000].
         */
        void canonize()
        {
	  int const UsecPerSec = 1000000;
	  int offset = microseconds / UsecPerSec;
	  seconds      += offset;
	  microseconds -= offset * UsecPerSec;

	  if(microseconds < 0) {
	    seconds--;
	    microseconds += UsecPerSec;
	  }
        }
#endif
    };
}

#endif
