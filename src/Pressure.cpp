#include "Pressure.hpp"

#include "Float.hpp"

namespace base {

Pressure::Pressure() : pascal(unknown<float>())
{

}

Pressure Pressure::fromPascal(float pascal)
{
    Pressure result;
    result.pascal = pascal;
    return result;
}

Pressure Pressure::fromBar(float bar)
{
    return fromPascal(bar * 100000);
}

Pressure Pressure::fromPSI(float psi)
{
    return fromPascal(psi * 6894.75729);
}

float Pressure::toPa() const
{
    return pascal;
}

float Pressure::toBar() const
{
    return pascal / 100000;
}

float Pressure::toPSI() const
{
    return pascal / 6894.75729;
}

} //end namespace base




