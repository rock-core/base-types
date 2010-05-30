#ifndef BASE_TIMEMARK_H__
#define BASE_TIMEMARK_H__

#include "time.h"

namespace base {

struct TimeMark
{
    std::string label;
    Time mark;

    TimeMark(const std::string& label) : label(label), mark( Time::now() ) {};

    /** Return the time that has passed since the recorded time and now */
    Time passed()
    {
	return (Time::now() - mark);
    }
};

}

inline std::ostream &operator<<(std::ostream &stream, base::TimeMark ob)
{
  stream << ob.passed() << "s since " << ob.label;
  return stream;
}

#endif
