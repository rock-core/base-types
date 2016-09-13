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

    TimeMark(const std::string& label);

    /** Return the time that has passed since the recorded time and now */
    Time passed() const;

    clock_t cycles() const;
};


std::ostream &operator<<(std::ostream &stream, const TimeMark &ob);

} // namespace base


#endif
