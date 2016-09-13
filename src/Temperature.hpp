#ifndef __BASE_TEMPERATURE_HH__
#define __BASE_TEMPERATURE_HH__

#include <complex>      // std::abs

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
    Temperature(); 
    
protected:
    explicit Temperature( double kelvin );

    
public:
    /**
     * static conversion from kelvin to celsius
     * @param kelvin temperature in kelvin
     * @result temperature in celsius
     */
    static double kelvin2Celsius( double kelvin );

    /**
     * static conversion from celsius to kelvin
     * @param celsius temperature in celsius
     * @result temperature in kelvin
     */
    static double celsius2Kelvin( double celsius );


    /** 
     * use this method to get temperature from Kelvin.
     * @return representation of the given temperature.
     * @param kelvin - temperature in Kelvin.
     */
    static Temperature fromKelvin( double kelvin );


    /** 
     * use this method to get temperature from Celsius
     * @return representation of the given temperature.
     * @param celsius - temperature in celsius.
     */
    static Temperature fromCelsius( double celsius );
    
    /**
     * @return canonical value of the temperature in kelvin
     */
    double getKelvin() const ;

    /**
     * @return canonical value of the temperature in celsius
     */
    double getCelsius() const;

    /**
     * @return true if the temperature is insight the given interval
     */
    bool isInRange(const Temperature &left_limit,const Temperature &right_limit) const;
    
    /**
     * compare two temperatures for approximate equality
     * @param other - temperature to compare
     * @param prec - precision interval in kelvin
     * @return true if temperature is approximately equal 
     */
    bool isApprox( Temperature other, double prec = 1e-5 ) const;

    void operator=(const Temperature &other);
    
    bool operator==(const Temperature &other ) const;
    
    bool operator<(const Temperature &other ) const;
    
    bool operator>(const Temperature &other ) const;

};

Temperature operator+( Temperature a, Temperature b );

Temperature operator-( Temperature a, Temperature b );

Temperature operator*( Temperature a, double b );

Temperature operator*( double a, Temperature b );

std::ostream& operator << (std::ostream& os, Temperature temperature);


}

#endif
