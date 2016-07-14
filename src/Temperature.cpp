#include "Temperature.hpp"

#include <boost/format.hpp>
#include <base/Float.hpp>

base::Temperature::Temperature() : kelvin(base::unknown<double>())
{
    
}

base::Temperature::Temperature(double kelvin) : kelvin(kelvin) 
{

}

double base::Temperature::kelvin2Celsius(double kelvin)
{
    return kelvin - 273.15;
}

double base::Temperature::celsius2Kelvin(double celsius)
{
    return celsius + 273.15;
}

base::Temperature base::Temperature::fromKelvin(double kelvin)
{
    return Temperature( kelvin );
}

base::Temperature base::Temperature::fromCelsius(double celsius)
{
    return Temperature( celsius + 273.15);
}

double base::Temperature::getKelvin() const
{
    return kelvin;
}

double base::Temperature::getCelsius() const
{
    return kelvin - 273.15;
}

bool base::Temperature::isApprox(base::Temperature other, double prec) const
{
    return std::abs( other.kelvin - kelvin ) < prec;
}

void base::Temperature::operator=(const base::Temperature& other)
{
    kelvin = other.kelvin;
}

bool base::Temperature::operator==(const base::Temperature& other) const
{
    return this->kelvin == other.kelvin;
}

bool base::Temperature::operator<(const base::Temperature& other) const
{
    return this->kelvin < other.kelvin;
}

bool base::Temperature::operator>(const base::Temperature& other) const
{
    return this->kelvin > other.kelvin;
}

base::Temperature base::operator+(base::Temperature a, base::Temperature b)
{
    return Temperature::fromKelvin( a.getKelvin() + b.getKelvin() );
}

base::Temperature base::operator-(base::Temperature a, base::Temperature b)
{
    return Temperature::fromKelvin( a.getKelvin() - b.getKelvin() );
}

base::Temperature base::operator*(base::Temperature a, double b)
{
    return Temperature::fromKelvin( a.getKelvin() * b );
}

base::Temperature base::operator*(double a, base::Temperature b)
{
    return Temperature::fromKelvin( a * b.getKelvin() );
}

std::ostream& base::operator<<(std::ostream& os, base::Temperature temperature)
{
    os << temperature.getCelsius() << boost::format("[%3.1f celsius]");
    return os;
}

bool base::Temperature::isInRange(const base::Temperature &left_limit, const base::Temperature &right_limit) const
{
    if((right_limit-left_limit).kelvin < 0)
        return !isInRange(right_limit,left_limit);
    if((*this -left_limit).getKelvin() >= 0 && (right_limit -*this).getKelvin() >= 0) 
        return true;
    return false;
}
















