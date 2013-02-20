#ifndef BASE_TIMEMARK_H__
#define BASE_TIMEMARK_H__

#include "time.h"

namespace base {

struct TimeMark
{
    std::string label;
    Time mark;
    clock_t clock;

    TimeMark(const std::string& label) : label(label), mark( Time::now() ), clock( ::clock() ) {};

    /** Return the time that has passed since the recorded time and now */
    Time passed()
    {
	return (Time::now() - mark);
    }

    clock_t cycles()
    {
	return ::clock() - clock;
    }
};

}

inline std::ostream &operator<<(std::ostream &stream, base::TimeMark ob)
{
    stream << ob.cycles() << "cyc (" << ob.passed() << "s) since " << ob.label;
    return stream;
}


#endif
