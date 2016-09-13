#include "Temperature.hpp"

#include <boost/format.hpp>
#include <base/Float.hpp>

namespace base {

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

Temperature operator+(Temperature a, Temperature b)
{
    return Temperature::fromKelvin( a.getKelvin() + b.getKelvin() );
}

Temperature operator-(Temperature a, Temperature b)
{
    return Temperature::fromKelvin( a.getKelvin() - b.getKelvin() );
}

Temperature operator*(Temperature a, double b)
{
    return Temperature::fromKelvin( a.getKelvin() * b );
}

Temperature operator*(double a, Temperature b)
{
    return Temperature::fromKelvin( a * b.getKelvin() );
}

std::ostream& operator<<(std::ostream& os, Temperature temperature)
{
    os << temperature.getCelsius() << boost::format("[%3.1f celsius]");
    return os;
}

bool Temperature::isInRange(const Temperature &left_limit, const Temperature &right_limit) const
{
    if((right_limit-left_limit).kelvin < 0)
        return !isInRange(right_limit,left_limit);
    if((*this -left_limit).getKelvin() >= 0 && (right_limit -*this).getKelvin() >= 0) 
        return true;
    return false;
}


} //end namespace base


