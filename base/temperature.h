#ifndef __BASE_TEMPERATURE_HH__
#define __BASE_TEMPERATURE_HH__

#include <boost/format.hpp>
#include <base/eigen.h>

namespace base
{

/** 
 * This class represents a temperature, and can be used instead of double for
 * convenience. The class has a canonical representation of the temperature in
 * kelvin.
 */
class Temperature
{
public:
    /** 
     * temperature in kelvins
     * 
     *
     * @note don't use this value directly. It's only public to allow this class
     * to be used as an interface type.
     */
    double kelvin;

    /** 
     * default constructor, which will leave the temperature uninitialized.
     */
    Temperature() {}
    
protected:
    explicit Temperature( double kelvin ) : kelvin(kelvin) 
    { 
	
    }

    
public:
    /**
     * static conversion from kelvin to celsius
     * @param kelvin temperature in kelvin
     * @result temperature in celsius
     */
    static inline double kelvin2Celsius( double kelvin )
    {
	return kelvin - 273.15;
    }

    /**
     * static conversion from celsius to kelvin
     * @param celsius temperature in celsius
     * @result temperature in kelvin
     */
    static inline double celsius2Kelvin( double celsius )
    {
	return celsius + 273.15;
    }

    /** 
     * use this method to get temperature from Kelvin.
     * @return representation of the given temperature.
     * @param kelvin - temperature in Kelvin.
     */
    static inline Temperature fromKelvin( double kelvin )
    {
	return Temperature( kelvin );
    }

    /** 
     * use this method to get temperature from Celsius
     * @return representation of the given temperature.
     * @param celsius - temperature in celsius.
     */
    static inline Temperature fromCelsius( double celsius )
    {
	return Temperature( celsius + 273.15);
    }

    /**
     * @return canonical value of the temperature in kelvin
     */
    double inline getKelvin() const 
    {
	return kelvin;
    }

    /**
     * @return canonical value of the temperature in celsius
     */
    double inline getCelsius() const
    {
	return kelvin - 273.15;
    }

    /**
     * @return true if the temperature is insight the given interval
     */
    bool inline isInRange(const Temperature &left_limit,const Temperature &right_limit) const;
    
    /**
     * compare two temperatures for approximate equality
     * @param other - temperature to compare
     * @param prec - precision interval in kelvin
     * @return true if temperature is approximately equal 
     */
    bool inline isApprox( Temperature other, double prec = 1e-5 ) const
    {
	return fabs( other.kelvin - kelvin ) < prec;
    }

    void operator=(const Temperature &other)
    {
        kelvin = other.kelvin;
    }
    
    inline bool operator==(const Temperature &other ) const
    {
        return this->kelvin == other.kelvin;
    }
    
    inline bool operator<(const Temperature &other ) const
    {
        return this->kelvin < other.kelvin;
    }
    
    inline bool operator>(const Temperature &other ) const
    {
        return this->kelvin > other.kelvin;
    }

};

static inline Temperature operator+( Temperature a, Temperature b )
{
    return Temperature::fromKelvin( a.getKelvin() + b.getKelvin() );
}

static inline Temperature operator-( Temperature a, Temperature b )
{
    return Temperature::fromKelvin( a.getKelvin() - b.getKelvin() );
}

static inline Temperature operator*( Temperature a, double b )
{
    return Temperature::fromKelvin( a.getKelvin() * b );
}

static inline Temperature operator*( double a, Temperature b )
{
    return Temperature::fromKelvin( a * b.getKelvin() );
}



static inline std::ostream& operator << (std::ostream& os, Temperature temperature)
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

}

#endif
