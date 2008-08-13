#ifndef DFKI_BASE_TYPES_H__
#define DFKI_BASE_TYPES_H__

#ifndef __orogen
#include <sys/time.h>
#include <time.h>
#endif

namespace DFKI {
    struct Time {
        int seconds;
        int microseconds;

#ifndef __orogen
        Time()
            : seconds(0), microseconds(0) {}

        explicit Time(int seconds, int microseconds = 0)
            : seconds(seconds), microseconds(microseconds) {}

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
        }
        Time operator + (Time const& ts) const
        {
            Time result;
            result.seconds      = seconds + ts.seconds;
            result.microseconds = microseconds + ts.microseconds;
            result.canonize();
        }

        bool isNull() const { return seconds == 0 && microseconds == 0; }
        timeval toTimeval() const
        {
            timeval tv = { seconds, microseconds };
            return tv;
        }

    private:
        void canonize()
        {
            int const UsecPerSec = 1000000;
            int offset = microseconds / UsecPerSec;
            seconds      += offset;
            microseconds -= offset * UsecPerSec;
        }
#endif
    };
}

#endif
