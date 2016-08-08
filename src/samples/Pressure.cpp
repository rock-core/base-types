#include "Pressure.hpp"

base::samples::Pressure base::samples::Pressure::fromPascal(const base::Time& time, float pascal)
{
    return Pressure(time, base::Pressure::fromPascal(pascal));
}

base::samples::Pressure base::samples::Pressure::fromBar(const base::Time& time, float bar)
{
    return Pressure(time, base::Pressure::fromBar(bar));
}

base::samples::Pressure base::samples::Pressure::fromPSI(const base::Time& time, float psi)
{
    return Pressure(time, base::Pressure::fromPSI(psi));
}
