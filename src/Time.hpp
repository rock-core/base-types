#ifndef BASE_TIME_H__
#define BASE_TIME_H__

#include <cstdlib>
#include <ostream>
#include <stdint.h>
#include <vector>

namespace base
{
    struct Time
    {
    private:
        explicit Time(int64_t _microseconds);

    public:
        int64_t microseconds;

        static const int UsecPerSec = 1000000LL;

        enum Resolution { Seconds = 1, Milliseconds = 1000, Microseconds = 1000000 };

        Time();

        static std::string DEFAULT_FORMAT;

    public:
        /** Returns the current time */
        static Time now();

        bool operator < (Time const& ts) const;
        bool operator > (Time const& ts) const;
        bool operator == (Time const& ts) const;
        bool operator != (Time const& ts) const;
        bool operator >= (Time const& ts) const;
        bool operator <= (Time const& ts) const;
        Time operator - (Time const& ts) const;
        Time operator + (Time const& ts) const;
        Time operator / (int divider) const;
        Time operator * (double factor) const;

        /** True if this time is zero */
        bool isNull() const;

        /** Converts this time as a timeval object */
        timeval toTimeval() const;

        /** Converts this time to a vector containing the number of
         * microseconds, milliseconds, seconds, minutes, hours and days
         * contained in that order.
         * @example: for 86400000000 microseconds the vector contains:
         * {0, 0, 0, 0, 0, 1}
         */
        std::vector<int> toTimeValues() const;

        /** Convert time into a string
         * \param resolution Resolution which the string should present
         * \param mainFormat Main format to use -- this is passed to strftime and appended by ':' plus the
         *     below seconds resolution if requested by the resolution argument,
         *     and additionally suffixed by '\%z' (see documentation of strftime)
         *     the current timezone's offset to UTC
         **/
        std::string toString(base::Time::Resolution resolution = Microseconds,
                const std::string& mainFormat = base::Time::DEFAULT_FORMAT) const;

        /**
          * Get the current timezone's offset to UTC - corresponding
          * to strftime's format field '\%z'
          * \param when Time in seconds since the Epoch, 1970-01-01 00:00:00 +0000 (UTC)
          * \return timezone offset in seconds for time specified by \p when
          */
        static int64_t getTimezoneOffset(time_t when);

        /**
          * Convert a string representing the numeric timezone
          * (see strftime documentation of format field '\%z') into seconds
          * \param tzInfo numeric timezone (as offset to UTC) in the format +hhmm or -hhmm
          * \return time offset in seconds to UTC
          */
        static int64_t tzInfoToSeconds(const std::string& tzInfo);

        /** Returns this time as a fractional number of seconds */
        double toSeconds() const;

        /** Returns this time as an integer number of milliseconds (thus dropping the microseconds) */
        int64_t toMilliseconds() const;

        /** Returns this time as an integer number of microseconds */
        int64_t toMicroseconds() const;

        static Time fromMicroseconds(int64_t value);

        static Time fromMilliseconds(int64_t value);

        static Time fromSeconds(int64_t value);

        static Time fromSeconds(int value);

        static Time fromSeconds(int64_t value, int microseconds);

        static Time fromSeconds(double value);

        /** Returns the maximum time value possible */
        static Time max();

        /**
          * \brief Create time from broken-down time values specified in the
          *        local time zone
          *
          * Creates a time object from the time values (year, month, day ...)
          * given as integer values. This function can be used when the time
          * values are only available as seperated values in numerical form.
          *
          * The specified values are expected in the local time zone
          *
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
        static Time fromTimeValues(
            int year, int month, int day, int hour, int minute, int seconds,
            int millis, int micros
        );

        /**
        * Create a time object from an input string, interpreted as local time
        *
        * \param stringTime String describing the time
        *                   If the string ends with the offset to UTC '\%z' (\see
        *                   documentation of strftime), this offset is applied to
        *                   return a UTC based time object.
        *                   If the string omits the '\%z' suffix it is interpreted as a local time.
        * \param resolution Set to a resolution higher than Secs if a (non-standard)
        *                   msec or usec field is present. The milliseconds or
        *                   microseconds are added after the time string, separated by
        *                   a colon, e.g. 2012-06-14--12.05.06Z:00100
        * \param mainFormat valid main format for strptime, e.g."%Y%m%d-%H:%M:%S". Note
        *                   that the timezone field '\%Z' is parsed but basically
        *                   ignored.
        *                   The default format converts the value returned by
        *                   fromString (i.e. Time::fromString(time.toString()) is
        *                   the identity).
        * \returns If the input string ends with a time offset representation of the form
        *           +HHMM (+0230 for 2:30 east of UTC), the value returned is
        *           the UTC time. Otherwise, the string is interpreted as a time
        *           in the local time zone and converted to UTC accordingly.
        * \throws std::runtime_error on failure such as a mismatching format
        */
        static Time fromString(
            const std::string& stringTime,
            Resolution resolution = Microseconds,
            const std::string& mainFormat = base::Time::DEFAULT_FORMAT
        );
    };

    std::ostream& operator << (std::ostream& io, base::Time const& time);
}

#endif
