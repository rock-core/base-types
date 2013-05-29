#ifndef __BASE_ANGLE_HH__
#define __BASE_ANGLE_HH__

#include <boost/format.hpp>
#include <math.h>
#include <base/Eigen.hpp>

namespace base
{

/** 
 * This class represents an angle, and can be used instead of double for
 * convenience. The class has a canonical representation of the angle in
 * degrees, in the interval PI < rad <= PI. 
 */
class Angle
{
public:
    /** 
     * angle in radians.
     * this value will always be PI < rad <= PI
     *
     * @note don't use this value directly. It's only public to allow this class
     * to be used as an interface type.
     */
    double rad;

    /** 
     * default constructor, which will leave the angle uninitialized.
     */
    Angle() {}
    
protected:
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
    /**
     * static conversion from radians to degree
     * @param rad angle in radians
     * @result angle in degree
     */
    static inline double rad2Deg( double rad )
    {
	return rad / M_PI * 180.0;
    }

    /**
     * static conversion from degree to radians
     * @param deg angle in degree
     * @result angle in radians
     */
    static inline double deg2Rad( double deg )
    {
	return deg / 180.0 * M_PI;
    }

    /** Normalize an angular value in [-pi; pi] and returns it as double
     */
    static inline double normalizeRad( double rad )
    {
        return Angle(rad).rad;
    }

    /** 
     * use this method to get an angle from radians.
     * @return representation of the given angle.
     * @param rad - angle in radians.
     */
    static inline Angle fromRad( double rad )
    {
	return Angle( rad );
    }

    /** 
     * use this method to get an angle from degrees.
     * @return representation of the given angle.
     * @param deg - angle in degrees.
     */
    static inline Angle fromDeg( double deg )
    {
	return Angle( deg / 180.0 * M_PI );
    }

    /**
     * @return canonical value of the angle in radians
     */
    double inline getRad() const 
    {
	return rad;
    }

    /**
     * @return canonical value of the angle in degrees
     */
    double inline getDeg() const
    {
	return rad / M_PI * 180;
    }

    /**
     * @return true if the angle is insight the given interval
     */
    bool inline isInRange(const Angle &left_limit,const Angle &right_limit) const;
    /**
     * compare two angles for approximate equality
     * @param other - angle to compare
     * @param prec - precision interval in deg
     * @return true if angle is approximately equal 
     */
    bool inline isApprox( Angle other, double prec = 1e-5 ) const
    {
	return fabs( other.rad - rad ) < prec;
    }

    void operator=(const Angle &other)
    {
        rad = other.rad;
    }
    
    inline bool operator==(const Angle &other ) const
    {
        return this->rad == other.rad;
    }
    
    inline bool operator<(const Angle &other ) const
    {
        return this->rad < other.rad;
    }
    
    inline bool operator>(const Angle &other ) const
    {
        return this->rad > other.rad;
    }

    /** Computes the unsigned angle of the rotation that makes +a+ colinear with +b+
     */
    static Angle vectorToVector(const base::Vector3d& a, const base::Vector3d& b)
    {
        double dot = a.dot(b);
        double norm = a.norm() * b.norm();
        return fromRad(acos(dot / norm));
    }

    /** Computes the angle of the rotation that makes +a+ colinear with +b+
     *
     * Unlike vectorToVector(a, b), this method computes a signed angle, with the
     * @c positive vector defining the positive rotation direction. @c positive
     * is required to be of unit length.
     */
    static Angle vectorToVector(const base::Vector3d& a, const base::Vector3d& b, const base::Vector3d& positive)
    {
        double cos = a.dot(b) / (a.norm() * b.norm());

        bool is_positive = (a.cross(b).dot(positive) > 0);
        if (is_positive)
            return fromRad(acos(cos));
        else
            return fromRad(-acos(cos));
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

bool Angle::isInRange(const Angle &left_limit, const Angle &right_limit) const
{
    if((right_limit-left_limit).rad < 0)
        return !isInRange(right_limit,left_limit);
    if((*this -left_limit).getRad() >= 0 && (right_limit -*this).getRad() >= 0) 
        return true;
    return false;
}

}

#endif
