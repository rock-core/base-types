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
#include <stdexcept>
#include <assert.h>

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

        enum Resolution { Seconds = 1, Milliseconds = 1000, Microseconds = 1000000 };

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

	/** Convert time into a string
         * \param resolution Resolution which the string should present
         * \param mainFormat Main format to use -- this is passed to strftime and appended by ':' plus the
         *     below seconds resolution if requested by the resolution argument
         **/
	std::string toString(base::Time::Resolution resolution = Microseconds, const std::string& mainFormat = "%Y%m%d-%H:%M:%S") const
	{
            struct timeval tv = toTimeval();
            int uSecs = tv.tv_usec;

	    time_t when = tv.tv_sec;
	    struct tm *tm = localtime(&when); 

            char time[50];
            strftime(time,50, mainFormat.c_str(), tm);

            char buffer[57];
            switch(resolution)
            {
                case Seconds:
                    return std::string(time);
                case Milliseconds:
                    sprintf(buffer,"%s:%03d", time, (int) (uSecs/1000.0));
                    break;
                case Microseconds:
                    sprintf(buffer,"%s:%06d", time, uSecs);
                    break;
                default: 
                    assert(-1);
            }

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
	static Time fromMilliseconds(uint64_t value)
        { return Time(value * 1000); }
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


        /**
          * \brief Create time from int Time Values.
          * Creates a time object from the time values (year, month, day ...) given as integer values. This function can be used
          * when the time values are only available as seperated values in numerical form.
          * \param year The year as integer value. (should be 4 digits)
          * \param month The month of the year (1..12).
          * \param day Day of the month (1..31).
          * \param hour The hour of the day (since midnight 0..23).
          * \param minute The minutes after the hour (0..59). 
          * \param seconds The seconds after the minute (0..59)
          * \param millis Milliseconds after the last second (0..999)
          * \param micros Microseconds additional to the milliseconds (0..999)
          * \returns a Time object generated from the parameters.
          *
          */
        static Time fromTimeValues(int year, int month, int day, int hour, int minute, int seconds, int millis, int micros)
        {

            struct tm timeobj;
	    timeobj.tm_year = year - 1900;
            timeobj.tm_mon = month - 1;
            timeobj.tm_mday = day;
            timeobj.tm_hour = hour;
            timeobj.tm_min = minute;
            timeobj.tm_sec = seconds;
            timeobj.tm_isdst = -1;

            time_t tTime;
            tTime = mktime(&timeobj);

            int64_t timeVal =  static_cast<int64_t>(tTime);

            timeVal = timeVal * UsecPerSec;
            timeVal += millis * 1000 + micros;


            return Time(timeVal); 

        }
        

        /**
        * Create a time object from an input string, by default all parameters are set to convert the string returned
        * by toString back to a Time object. 
        * \param stringTime String describing the time
        * \param resolution Set to a resolution higher than Secs if a (non-standard) msec or usec field is present, i.e. the non standard field is separated by ':'
        * \param mainFormat valid format for strptime, e.g."%Y%m%d-%H:%M:%S" which the given time string has
        * \throws std::runtime_error on failure such as a mismatching format
        */
        static Time fromString(const std::string& stringTime, Resolution resolution = Microseconds, const std::string& mainFormat = "%Y%m%d-%H:%M:%S")
        {
            std::string mainTime = stringTime;
            int usecs = 0;
            if(resolution > Seconds)
            {
                size_t pos = stringTime.find_last_of(':');
                std::string mainTime = stringTime.substr(0,pos-1);
                std::string usecsString = stringTime.substr(pos+1);
                size_t usecsStringLength = usecsString.size();
                if( (usecsStringLength == 3 || usecsStringLength == 6) && !(usecsStringLength == 3 && resolution > Milliseconds))
                {
                    // string matches resolutions
                } else
                { 
                    throw std::runtime_error("base::Time::fromString failed - resolution does not match provided Time-String");
                }

                switch(resolution)
                {
                    case Milliseconds:
                        sscanf(usecsString.c_str(), "%03d", &usecs);
                        usecs = usecs*1000;
                        break;
                    case Microseconds:
                        sscanf(usecsString.c_str(), "%06d", &usecs);
                        break;
                    default:
                        assert(-1);
                }
            }

            struct tm tm;
            if(NULL == strptime(mainTime.c_str(), mainFormat.c_str(), &tm))
            {
                throw std::runtime_error("base::Time::fromString failed- Time-String '" + mainTime + "' did not match the given format '" + mainFormat +"'");
            }
            // " ... not set by strptime(); tells mktime() to determine 
            // whether daylight saving time is in effect ..."
            // (http://pubs.opengroup.org/onlinepubs/007904975/functions/strptime.html)
              
            tm.tm_isdst = -1; 
            time_t time = mktime(&tm);

            return Time(static_cast<int64_t>(time* UsecPerSec + usecs));
        }

    };

    inline std::ostream& operator << (std::ostream& io, base::Time const& time)
    {
	const int64_t microsecs = time.toMicroseconds();

	io << (microsecs / 1000000)
	   << std::setfill('0')
	   << "." << std::setw(3) << (std::llabs(microsecs) / 1000) % 1000
	   << "." << std::setw(3) << (std::llabs(microsecs) % 1000)
	   << std::setfill(' ');

	return io;
    }
}


#endif
