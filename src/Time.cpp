#include "Time.hpp"

#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <stdio.h>
#include <time.h>
#include <limits>
#include <regex>
#include <chrono>

using namespace std;
using namespace base;

string Time::DEFAULT_FORMAT = "%Y%m%d-%H:%M:%S";

Time::Time(int64_t _microseconds) : microseconds(_microseconds)
{

}

Time::Time() : microseconds(0)
{

}

Time Time::now()
{
    timeval t;
    gettimeofday(&t, 0);
    return Time(static_cast<int64_t>(t.tv_sec) * UsecPerSec + t.tv_usec);
}

Time Time::monotonic()
{
    // Note: C++11 statics are thread safe
    static auto monotonicClock = chrono::steady_clock();

    auto tp = monotonicClock.now().time_since_epoch();
    auto us = chrono::duration_cast<chrono::microseconds>(tp);
    return Time(us.count());
}

bool Time::operator<(const Time& ts) const
{
    return microseconds < ts.microseconds;
}

bool Time::operator>(const Time& ts) const
{
    return microseconds > ts.microseconds;
}

bool Time::operator==(const Time& ts) const
{
    return microseconds == ts.microseconds;
}

bool Time::operator!=(const Time& ts) const
{
    return !(*this == ts);
}

bool Time::operator>=(const Time& ts) const
{
    return !(*this < ts);
}

bool Time::operator<=(const Time& ts) const
{
    return !(*this > ts);
}

Time Time::operator-(const Time& ts) const
{
    return Time(microseconds - ts.microseconds);
}

Time Time::operator+(const Time& ts) const
{
    return Time(microseconds + ts.microseconds);
}

Time Time::operator/(int divider) const
{
    return Time(microseconds / divider);
}

Time Time::operator*(double factor) const
{
    return Time(microseconds * factor);
}

bool Time::isNull() const
{
    return microseconds == 0;
}

timeval Time::toTimeval() const
{
    timeval tv = { static_cast<time_t>(microseconds / UsecPerSec), static_cast<suseconds_t>(microseconds % UsecPerSec) };
    return tv;
}

vector<int> Time::toTimeValues() const
{
    int64_t microseconds = this->microseconds;

    int64_t days = microseconds / 86400000000ll;
    microseconds -= days * 86400000000ll;
    int64_t hours = microseconds / 3600000000ll;
    microseconds -= hours * 3600000000ll;
    int64_t minutes = microseconds / 60000000ll;
    microseconds -= minutes * 60000000ll;
    int64_t seconds = microseconds / 1000000ll;
    microseconds -= seconds * 1000000ll;
    int64_t milliseconds = microseconds / 1000ll;
    microseconds -= milliseconds * 1000ll;

    vector<int> timeValues;
    timeValues.reserve(6);
    timeValues.push_back(static_cast<int>(microseconds));
    timeValues.push_back(static_cast<int>(milliseconds));
    timeValues.push_back(static_cast<int>(seconds));
    timeValues.push_back(static_cast<int>(minutes));
    timeValues.push_back(static_cast<int>(hours));
    timeValues.push_back(static_cast<int>(days));

    return timeValues;
}

string Time::toString(Time::Resolution resolution,
        const string& mainFormat) const
{
    struct timeval tv = toTimeval();
    int uSecs = tv.tv_usec;

    time_t when = tv.tv_sec;
    struct tm *tm = localtime(&when);

    char time[50];
    strftime(time, 50, mainFormat.c_str(), tm);

    char tzInfo[6];
    strftime(tzInfo, 6, "%z", tm);

    char buffer[57];
    switch(resolution)
    {
        case Seconds:
            sprintf(buffer,"%s%s", time, tzInfo);
            break;
        case Milliseconds:
            sprintf(buffer,"%s:%03d%s", time, (int) (uSecs/1000.0),tzInfo);
            break;
        case Microseconds:
            sprintf(buffer,"%s:%06d%s", time, uSecs,tzInfo);
            break;
        default:
            throw invalid_argument(
                "Time::toString(): invalid "
                "value in switch-statement");
    }
    return string(buffer);
}

double Time::toSeconds() const
{
    return static_cast<double>(microseconds) / UsecPerSec;
}

int64_t Time::toMilliseconds() const
{
    return microseconds / 1000;
}

int64_t Time::toMicroseconds() const
{
    return microseconds;
}

Time Time::fromMicroseconds(int64_t value)
{
    return Time(value);
}

Time Time::fromMilliseconds(int64_t value)
{
    return Time(value * 1000);
}

Time Time::fromSeconds(int64_t value)
{
    return Time(value * UsecPerSec);
}

Time Time::fromSeconds(int value)
{
    return Time(static_cast<int64_t>(value) * UsecPerSec);
}

Time Time::fromSeconds(int64_t value, int microseconds)
{
    return Time(value * UsecPerSec + static_cast<int64_t>(microseconds));
}

Time Time::fromSeconds(double value)
{
    int64_t seconds = value;
    return Time(seconds * UsecPerSec +
                static_cast<int64_t>(round((value - seconds) * UsecPerSec)));
}

Time Time::max()
{
    return Time(numeric_limits<int64_t>::max());
}

Time Time::fromTimeValues(int year, int month, int day,
                          int hour, int minute, int seconds, int millis, int micros)
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

Time Time::fromString(const string& stringTime, Time::Resolution resolution,
                      const string& mainFormat)
{
    string mainTime = stringTime;
    int32_t usecs = 0;

    // Check for %z suffix
    string tzInfo;
    regex tzPattern("(.*)([\\-\\+][0-9]{4}$)");
    smatch tzMatch;
    if(regex_match(stringTime, tzMatch, tzPattern))
    {
        tzInfo = tzMatch[2].str();
        mainTime = tzMatch[1].str();
    }

    if (resolution > Seconds)
    {
        size_t pos = mainTime.find_last_of(':');
        string usecsString = mainTime.substr(pos+1);
        size_t usecsStringLength = usecsString.size();
        bool match = (usecsStringLength == 6) ||
                     (usecsStringLength == 3 && resolution == Milliseconds);

        if (!match)
        {
            throw runtime_error(
                "Time::fromString: required resolution format does not match the given string '"
                    + stringTime + "' -- identified subseconds: '" + usecsString + "'"
            );
        }

        switch (resolution)
        {
            case Milliseconds:
                sscanf(usecsString.c_str(), "%03d", &usecs);
                usecs = usecs*1000;
                break;
            case Microseconds:
                sscanf(usecsString.c_str(), "%06d", &usecs);
                break;
            case Seconds:
                throw invalid_argument(
                    "Time::fromString(); "
                    "'Seconds' is an invalid case "
                    "here");
            default:
                throw invalid_argument("Time::fromString(): "
                                            "invalid value in "
                                            "switch-statement");
        }
    }

    struct tm tm;
    if (NULL == strptime(mainTime.c_str(), mainFormat.c_str(), &tm))
    {
        throw runtime_error(
            "Time::fromString failed: " + mainTime + "' did not match the given "
            "format '" + mainFormat +"'"
        );
    }

    time_t time;
    if(tzInfo.empty())
    {
        tm.tm_isdst = -1;
        time = mktime(&tm);
    } else
    {
        // We have to go through a few intermediate "time zones" to reach UTC
        // The problem we try to work around here is that mktime converts a broken-down time
        // in the local time zone to UTC, but what we have is a time in UTC and want to convert
        // it to UTC

        // 'tm' is the time represented by the string. It is `tzInfo` seconds away from UTC
        int64_t tzOffset = Time::tzInfoToSeconds(tzInfo);
        tm.tm_sec += tzOffset;

        // OK, now `tm` is the string's time in UTC. Note that mktime handles wraparounds
        tm.tm_isdst = -1;
        time = mktime(&tm);

        // Since mktime expected `tm` to be in local time, `time` is UTC with the local time
        // zone offset as it was at `tm` applied in opposite (e.g. time would be UTC-3 if
        // executed on a machine whose current time zone is UTC+3).
        // We have to correct with the local offset to finally get UTC
        int64_t localTimezoneOffset = Time::getTimezoneOffset(time);
        time -= localTimezoneOffset;
    }

    return Time(static_cast<int64_t>(time)*UsecPerSec + static_cast<int64_t>(usecs));
}

int64_t Time::getTimezoneOffset(time_t when)
{
    // gmtime returns UTC
    struct tm *tm = gmtime(&when);
    // " tm_isdst ... not set by strptime(); -1 tells mktime() to determine
    // whether daylight saving time is in effect ..."
    // (http://pubs.opengroup.org/onlinepubs/007904975/functions/strptime.html)
    tm->tm_isdst = -1;
    time_t localWhen = mktime(tm);
    return localWhen - when;
}

int64_t Time::tzInfoToSeconds(const string& tzInfo)
{
    int64_t tzOffset = 0;
    int hours;
    int minutes;
    int r = sscanf(tzInfo.c_str(), "%3d%2d",&hours, &minutes);
    if(r != 2 || tzInfo.size() != 5)
    {
        throw invalid_argument("base::Time::tzInfoToSeconds: parsing of "
                "timezone offset '" +tzInfo + "' failed");
    }
    tzOffset = hours*3600;
    if(tzOffset < 0)
    {
        tzOffset -= minutes*60;
    } else {
        tzOffset += minutes*60;
    }
    // adapt to same direction as timezone value
    return -tzOffset;
}


ostream& base::operator<<(ostream& io, const Time& time)
{
    const int64_t microsecs = time.toMicroseconds();

    io << (microsecs / 1000000)
        << setfill('0')
        << "." << setw(3) << (llabs(microsecs) / 1000) % 1000
        << "." << setw(3) << (llabs(microsecs) % 1000)
        << setfill(' ');

    return io;
}

