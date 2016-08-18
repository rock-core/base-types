#include "Pressure.hpp"

namespace base { namespace samples {

Pressure Pressure::fromPascal(const Time& time, float pascal)
{
    return Pressure(time, base::Pressure::fromPascal(pascal));
}

Pressure Pressure::fromBar(const Time& time, float bar)
{
    return Pressure(time, base::Pressure::fromBar(bar));
}

Pressure Pressure::fromPSI(const Time& time, float psi)
{
    return Pressure(time, base::Pressure::fromPSI(psi));
}

}} //end namespace base::samples