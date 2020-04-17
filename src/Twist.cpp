#include "Twist.hpp"

namespace base {

Twist::Twist()
{
    setNaN();
}

Twist::Twist(base::Vector3d linear, base::Vector3d angular) :
    linear(linear),
    angular(angular)
{
}

void Twist::setNaN()
{
    linear.setConstant(std::numeric_limits<double>::quiet_NaN());
    angular.setConstant(std::numeric_limits<double>::quiet_NaN());
}

void Twist::setZero()
{
    linear.setZero();
    angular.setZero();
}

bool Twist::isValid() const
{
    return base::isnotnan(linear) && base::isnotnan(angular);
}

Twist operator+(const Twist& a, const Twist& b)
{
    return Twist(a.linear  + b.linear, a.angular + b.angular);
}

Twist operator-(const Twist& a, const Twist& b)
{
    return Twist(a.linear  - b.linear, a.angular - b.angular);
}

}
