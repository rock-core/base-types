#ifndef __BASE_ANGLE_HH__
#define __BASE_ANGLE_HH__

#include <boost/format.hpp>

namespace base
{

class Angle
{
    // angle in radians 
    // will always be PI < rad <= PI
    double rad;
    
    explicit Angle( double rad ) : rad(rad) 
    { 
	canonize(); 
    }

    void canonize()
    {
	if( rad > M_PI || rad <= -M_PI )
	{
	    double intp;
	    const double side = copysign(M_PI,rad);
	    rad = -side + 2*M_PI * modf( (rad-side) / (2*M_PI), &intp ); 
	}
    }

public:
    static inline Angle fromRad( double rad )
    {
	return Angle( rad );
    }

    static inline Angle fromDeg( double deg )
    {
	return Angle( deg / 180.0 * M_PI );
    }

    double inline getRad() const 
    {
	return rad;
    }

    double inline getDeg() const
    {
	return rad / M_PI * 180;
    }

    bool inline isApprox( Angle other, double prec = 1e-5 )
    {
	return fabs( other.rad - rad ) < prec;
    }
};

static inline Angle operator+( Angle a, Angle b )
{
    return Angle::fromRad( a.getRad() + b.getRad() );
}

static inline Angle operator-( Angle a, Angle b )
{
    return Angle::fromRad( a.getRad() - b.getRad() );
}

static inline Angle operator*( Angle a, double b )
{
    return Angle::fromRad( a.getRad() * b );
}

static inline Angle operator*( double a, Angle b )
{
    return Angle::fromRad( a * b.getRad() );
}

static inline std::ostream& operator << (std::ostream& os, Angle angle)
{
    os << angle.getRad() << boost::format("[%3.1fdeg]") % angle.getDeg();
    return os;
}

}

#endif
