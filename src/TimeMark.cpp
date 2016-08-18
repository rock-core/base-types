#include "TimeMark.hpp"

namespace base {

TimeMark::TimeMark(const std::string& label) : label(label), mark( Time::now() ), clock( ::clock() )
{

}

Time TimeMark::passed() const
{
    return (Time::now() - mark);
}

clock_t TimeMark::cycles() const
{
    return ::clock() - clock;
}

std::ostream& operator<<(std::ostream& stream, const TimeMark& ob)
{
    stream << ob.cycles() << "cyc (" << ob.passed() << "s) since " << ob.label;
    return stream;
}


} //end namespace base
