#include "Acceleration.hpp"

namespace base {

Acceleration::Acceleration()
{
    setNaN();
}

Acceleration::Acceleration(base::Vector3d linear, base::Vector3d angular) :
    linear(linear),
    angular(angular)
{
}

void Acceleration::setNaN()
{
    linear.setConstant(std::numeric_limits<double>::quiet_NaN());
    angular.setConstant(std::numeric_limits<double>::quiet_NaN());
}

void Acceleration::setZero()
{
    linear.setZero();
    angular.setZero();
}

bool Acceleration::isValid() const
{
    return base::isnotnan(linear) && base::isnotnan(angular);
}

Acceleration operator+(const Acceleration& a, const Acceleration& b)
{
    return Acceleration(a.linear  + b.linear, a.angular + b.angular);
}

Acceleration operator-(const Acceleration& a, const Acceleration& b)
{
    return Acceleration(a.linear  - b.linear, a.angular - b.angular);
}

Acceleration operator*(const base::Vector6d& a, const Acceleration& b)
{
    return Acceleration(a.segment(0,3).cwiseProduct(b.linear), a.segment(3,3).cwiseProduct(b.angular));
}

}
