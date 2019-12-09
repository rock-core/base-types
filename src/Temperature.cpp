#include "Temperature.hpp"

#include <boost/format.hpp>
#include <base/Float.hpp>

using namespace std;
using namespace base;

Temperature::Temperature() : kelvin(unknown<double>())
{

}

Temperature::Temperature(double kelvin) : kelvin(kelvin)
{

}

double Temperature::kelvin2Celsius(double kelvin)
{
    return kelvin - 273.15;
}

double Temperature::celsius2Kelvin(double celsius)
{
    return celsius + 273.15;
}

Temperature Temperature::fromKelvin(double kelvin)
{
    return Temperature( kelvin );
}

Temperature Temperature::fromCelsius(double celsius)
{
    return Temperature( celsius + 273.15);
}

double Temperature::getKelvin() const
{
    return kelvin;
}

double Temperature::getCelsius() const
{
    return kelvin - 273.15;
}

bool Temperature::isApprox(Temperature other, double prec) const
{
    return std::abs( other.kelvin - kelvin ) < prec;
}

void Temperature::operator=(const Temperature& other)
{
    kelvin = other.kelvin;
}

bool Temperature::operator==(const Temperature& other) const
{
    return this->kelvin == other.kelvin;
}

bool Temperature::operator<(const Temperature& other) const
{
    return this->kelvin < other.kelvin;
}

bool Temperature::operator>(const Temperature& other) const
{
    return this->kelvin > other.kelvin;
}

Temperature base::operator+(Temperature a, Temperature b)
{
    return Temperature::fromKelvin( a.getKelvin() + b.getKelvin() );
}

Temperature base::operator-(Temperature a, Temperature b)
{
    return Temperature::fromKelvin( a.getKelvin() - b.getKelvin() );
}

Temperature base::operator*(Temperature a, double b)
{
    return Temperature::fromKelvin( a.getKelvin() * b );
}

Temperature base::operator*(double a, Temperature b)
{
    return Temperature::fromKelvin( a * b.getKelvin() );
}

std::ostream& base::operator<<(std::ostream& os, Temperature temperature)
{
    os << boost::format("[%3.1f celsius]") % temperature.getCelsius();
    return os;
}

bool Temperature::isInRange(const Temperature &left_limit, const Temperature &right_limit) const
{
    double min = left_limit.kelvin;
    double max = right_limit.kelvin;
    if (min > max) {
        swap(min, max);
    }
    return (min <= kelvin && kelvin <= max);
}
