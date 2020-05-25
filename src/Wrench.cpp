#include "Wrench.hpp"

namespace base{

Wrench::Wrench()
{
    setNaN();
}

Wrench::Wrench(base::Vector3d force, base::Vector3d torque) :
    force(force),
    torque(torque)
{
}

void Wrench::setNaN()
{
    force.setConstant(std::numeric_limits<double>::quiet_NaN());
    torque.setConstant(std::numeric_limits<double>::quiet_NaN());
}

void Wrench::setZero()
{
    force.setZero();
    torque.setZero();
}

bool Wrench::isValid() const
{
    return base::isnotnan(force) && base::isnotnan(torque);
}

Wrench operator+(const Wrench& a, const Wrench& b)
{
    return Wrench(a.force  + b.force, a.torque + b.torque);
}

Wrench operator-(const Wrench& a, const Wrench& b)
{
    return Wrench(a.force  - b.force, a.torque - b.torque);
}

}
