#include "Time.hpp"

#include <sys/time.h>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <stdio.h>
#include <time.h>

base::Time::Time(int64_t _microseconds) : microseconds(_microseconds)
{

}

base::Time::Time() : microseconds(0)
{

}

base::Time base::Time::now()
{
    timeval t;
    gettimeofday(&t, 0);
    return Time(static_cast<int64_t>(t.tv_sec) * UsecPerSec + t.tv_usec);
}

bool base::Time::operator<(const base::Time& ts) const
{
    return microseconds < ts.microseconds;
}

bool base::Time::operator>(const base::Time& ts) const
{
    return microseconds > ts.microseconds;
}

bool base::Time::operator==(const base::Time& ts) const
{
    return microseconds == ts.microseconds;
}

bool base::Time::operator!=(const base::Time& ts) const
{
    return !(*this == ts);
}

bool base::Time::operator>=(const base::Time& ts) const
{
    return !(*this < ts);
}

bool base::Time::operator<=(const base::Time& ts) const
{
    return !(*this > ts); 
}

base::Time base::Time::operator-(const base::Time& ts) const
{
    return Time(microseconds - ts.microseconds);
}

base::Time base::Time::operator+(const base::Time& ts) const
{
    return Time(microseconds + ts.microseconds);
}

base::Time base::Time::operator/(int divider) const
{
    return Time(microseconds / divider);
}

base::Time base::Time::operator*(double factor) const
{
    return Time(microseconds * factor);
}

bool base::Time::isNull() const
{
    return microseconds == 0;
}

timeval base::Time::toTimeval() const
{
    timeval tv = { static_cast<time_t>(microseconds / UsecPerSec), static_cast<suseconds_t>(microseconds % UsecPerSec) };
    return tv;
}

std::string base::Time::toString(base::Time::Resolution resolution, const std::string& mainFormat) const
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
            throw std::invalid_argument(
                "base::Time::toString(): invalid "
                "value in switch-statement");
    }

    return std::string(buffer);
}

double base::Time::toSeconds() const
{
    return static_cast<double>(microseconds) / UsecPerSec;
}

int64_t base::Time::toMilliseconds() const
{
    return microseconds / 1000;
}

int64_t base::Time::toMicroseconds() const
{
    return microseconds;
}

base::Time base::Time::fromMicroseconds(uint64_t value)
{
    return Time(value);
}

base::Time base::Time::fromMilliseconds(uint64_t value)
{
    return Time(value * 1000);
}

base::Time base::Time::fromSeconds(int64_t value)
{
    return Time(value * UsecPerSec); 
}

base::Time base::Time::fromSeconds(int value)
{
    return Time(static_cast<int64_t>(value) * UsecPerSec);
}

base::Time base::Time::fromSeconds(int64_t value, int microseconds)
{
    return Time(value * UsecPerSec + static_cast<int64_t>(microseconds));
}

base::Time base::Time::fromSeconds(double value)
{
    int64_t seconds = value;
    return Time(seconds * UsecPerSec + static_cast<int64_t>(round((value - seconds) * UsecPerSec)));
}

base::Time base::Time::fromTimeValues(int year, int month, int day, int hour, int minute, int seconds, int millis, int micros)
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

base::Time base::Time::fromString(const std::string& stringTime, base::Time::Resolution resolution, const std::string& mainFormat)
{
    std::string mainTime = stringTime;
    int32_t usecs = 0;
    if(resolution > Seconds)
    {
        size_t pos = stringTime.find_last_of(':');
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
            case Seconds:
                throw std::invalid_argument(
                    "base::Time::fromString(); "
                    "'Seconds' is an invalid case "
                    "here");
            default:
                throw std::invalid_argument("base::Time::fromString(): "
                                            "invalid value in "
                                            "switch-statement");
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

    return Time(static_cast<int64_t>(time)*UsecPerSec + static_cast<int64_t>(usecs));
}


std::ostream& base::operator<<(std::ostream& io, const base::Time& time)
{
    const int64_t microsecs = time.toMicroseconds();

    io << (microsecs / 1000000)
        << std::setfill('0')
        << "." << std::setw(3) << (std::llabs(microsecs) / 1000) % 1000
        << "." << std::setw(3) << (std::llabs(microsecs) % 1000)
        << std::setfill(' ');

    return io;
}

















