#include "TimeMark.hpp"

base::TimeMark::TimeMark(const std::__cxx11::string& label) : label(label), mark( Time::now() ), clock( ::clock() )
{

}

base::Time base::TimeMark::passed() const
{
    return (Time::now() - mark);
}

clock_t base::TimeMark::cycles() const
{
    return ::clock() - clock;
}

std::ostream& base::operator<<(std::ostream& stream, const base::TimeMark& ob)
{
    stream << ob.cycles() << "cyc (" << ob.passed() << "s) since " << ob.label;
    return stream;
}



