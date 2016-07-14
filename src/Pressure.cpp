#include "Pressure.hpp"

#include "Float.hpp"

base::Pressure::Pressure() : pascal(base::unknown<float>())
{

}

base::Pressure base::Pressure::fromPascal(float pascal)
{
    Pressure result;
    result.pascal = pascal;
    return result;
}

base::Pressure base::Pressure::fromBar(float bar)
{
    return fromPascal(bar * 100000);
}

base::Pressure base::Pressure::fromPSI(float psi)
{
    return fromPascal(psi * 6894.75729);
}

float base::Pressure::toPa() const
{
    return pascal;
}

float base::Pressure::toBar() const
{
    return pascal / 100000;
}

float base::Pressure::toPSI() const
{
    return pascal / 6894.75729;
}






