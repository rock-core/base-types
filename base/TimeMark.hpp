#ifndef BASE_TIMEMARK_H__
#define BASE_TIMEMARK_H__

#include <string>
#include <base/Time.hpp>

namespace base {

struct TimeMark
{
    std::string label;
    Time mark;
    clock_t clock;

    TimeMark(const std::string& label) : label(label), mark( Time::now() ), clock( ::clock() ) {};

    /** Return the time that has passed since the recorded time and now */
    Time passed() const
    {
	return (Time::now() - mark);
    }

    clock_t cycles() const
    {
	return ::clock() - clock;
    }
};


inline std::ostream &operator<<(std::ostream &stream, const TimeMark &ob)
{
    stream << ob.cycles() << "cyc (" << ob.passed() << "s) since " << ob.label;
    return stream;
}

} // namespace base


#endif
