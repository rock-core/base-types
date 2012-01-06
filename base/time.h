#ifndef BASE_TIME_H__
#define BASE_TIME_H__

#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <cstdlib>
#include <math.h>
#include <ostream>
#include <iomanip>

namespace base
{
    struct Time
    {
    private:
        explicit Time(int64_t microseconds)
            : microseconds(microseconds) { }
    public:
        int64_t microseconds;

	static const int UsecPerSec = 1000000LL;

        Time()
            : microseconds(0) {}

    public:
        /** Returns the current time */
        static Time now() {
            timeval t;
            gettimeofday(&t, 0);
            return Time(static_cast<int64_t>(t.tv_sec) * UsecPerSec + t.tv_usec);
        }

        bool operator < (Time const& ts) const
        { return microseconds < ts.microseconds; }
        bool operator > (Time const& ts) const
        { return microseconds > ts.microseconds; }
        bool operator == (Time const& ts) const
        { return microseconds == ts.microseconds; }
        bool operator != (Time const& ts) const
        { return !(*this == ts); }
        bool operator >= (Time const& ts) const
        { return !(*this < ts); }
        bool operator <= (Time const& ts) const
        { return !(*this > ts); }
        Time operator - (Time const& ts) const
        { return Time(microseconds - ts.microseconds); }
        Time operator + (Time const& ts) const
        { return Time(microseconds + ts.microseconds); }
        Time operator / (int divider) const
        { return Time(microseconds / divider); }
        Time operator * (double factor) const
        { return Time(microseconds * factor); }

        /** True if this time is zero */
        bool isNull() const { return microseconds == 0; }

        /** Converts this time as a timeval object */
        timeval toTimeval() const
        {
            timeval tv = { microseconds / UsecPerSec, microseconds % UsecPerSec };
            return tv;
        }

	/** Convert his time into a string object */
	std::string toString() const
	{
            struct timeval tv = toTimeval();
            int milliSecs = tv.tv_usec/1000;

	    time_t when = tv.tv_sec;
	    struct tm *tm = localtime(&when); 

            char time[25];
            strftime(time, 25, "%Y%m%d-%H:%M:%S", tm);

	    char buffer[25];
	    sprintf(buffer,"%s:%03d", time, milliSecs);
	    return std::string(buffer);
	}

        /** Returns this time as a fractional number of seconds */
        double toSeconds() const
        { return static_cast<double>(microseconds) / UsecPerSec; }
        /** Returns this time as an integer number of milliseconds (thus dropping the microseconds) */
        int64_t toMilliseconds() const
        { return microseconds / 1000; }
        /** Returns this time as an integer number of microseconds */
        int64_t toMicroseconds() const
        { return microseconds; }
        static Time fromMicroseconds(uint64_t value)
        { return Time(value); }
        static Time fromSeconds(int64_t value)
        { return Time(value * UsecPerSec); }
        static Time fromSeconds(int value)
        { return Time(static_cast<int64_t>(value) * UsecPerSec); }
        static Time fromSeconds(int64_t value, int microseconds)
        { return Time(value * UsecPerSec + static_cast<int64_t>(microseconds)); }
        static Time fromSeconds(double value)
        {
            int64_t seconds = value;
            return Time(seconds * UsecPerSec + static_cast<int64_t>(round((value - seconds) * UsecPerSec)));
        }
    };
}

inline std::ostream& operator << (std::ostream& io, base::Time const& time)
{
    const int64_t microsecs = time.toMicroseconds();

    io << (microsecs / 1000000)
        << std::setfill('0')
	<< "." << std::setw(3) << (std::abs(microsecs) / 1000) % 1000 
	<< "." << std::setw(3) << (std::abs(microsecs) % 1000)
	<< std::setfill(' ');

    return io;
}

#endif
