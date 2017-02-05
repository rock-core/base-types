#ifndef __BASE_ANGLE_HH__
#define __BASE_ANGLE_HH__

#include <math.h>
#include <iostream>
#include <base/Float.hpp>
#include <base/Eigen.hpp>

#include <vector>

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
     * default constructor, which will initialize the value to unknown (NaN)
     */
    Angle() : rad(base::unknown<double>()) {}
    
protected:
    explicit Angle( double _rad ) : rad(_rad)
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

    /** Use this method to represent an unknown angle
     */
    static inline Angle unknown()
    {
        Angle result;
        result.rad = base::unknown<double>();
        return result;
    }

    /** Returns the minimum angle possible after normalization
     *  @return representation of the minimum angle
     */
    static inline Angle Min()
    {
        return Angle( nextafter(-M_PI, 0) );
    }

    /** Returns the maximum angle possible after normalization
     *  @return representation of the maximum angle
     */
    static inline Angle Max()
    {
        return Angle(M_PI);
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
     * compare two angles for approximate equality
     * @param other - angle to compare
     * @param prec - precision interval in deg
     * @return true if angle is approximately equal 
     */
    bool inline isApprox( Angle other, double prec = 1e-5 ) const
    {
        return fabs(Angle(other.rad - rad).getRad()) < prec;
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

    inline bool operator<=(const Angle &other ) const
    {
        return this->rad <= other.rad;
    }

    inline bool operator>=(const Angle &other ) const
    {
        return this->rad >= other.rad;
    }

    inline Angle &operator+=(const Angle &other )
    {
        this->rad += other.rad;
        canonize();
        return *this;
    }

    inline Angle &operator-=(const Angle &other )
    {
        this->rad -= other.rad;
        canonize();
        return *this;
    }
    
    inline Angle operator+( const Angle &other ) const
    {
        return Angle::fromRad( getRad() + other.getRad() );
    }

    inline Angle operator-( const Angle &other ) const
    {
        return Angle::fromRad( getRad() - other.getRad() );
    }

    inline Angle operator*( const Angle &other ) const
    {
        return Angle::fromRad( getRad() * other.getRad() );
    }

    inline Angle operator*( const double &val ) const
    {
        return Angle::fromRad( getRad() * val );
    }

    /**
     * Returns a new angle which is the inverse of tis object.
     * */
    inline Angle flipped() const
    {
        return Angle(rad).flip();
    }
    
    /**
     * Inverts the current angle
     * */
    inline Angle &flip()
    {
        if(rad < 0)
            rad += M_PI;
        else
            rad -=M_PI;
        return *this;
    }

    /** Computes the unsigned angle of the rotation that makes +a+ colinear with +b+
     */
    static Angle vectorToVector(const base::Vector3d& a, const base::Vector3d& b);


    /** Computes the angle of the rotation that makes +a+ colinear with +b+
     *
     * Unlike vectorToVector(a, b), this method computes a signed angle, with the
     * @c positive vector defining the positive rotation direction. @c positive
     * is required to be of unit length.
     */
    static Angle vectorToVector(const base::Vector3d& a, const base::Vector3d& b, const base::Vector3d& positive);
};

static inline Angle operator*( double a, Angle b )
{
    return Angle::fromRad( a * b.getRad() );
}

std::ostream& operator << (std::ostream& os, Angle angle);

/**
 * This class represents a Segment of a Circle. 
 * This is primary a helper class for doing test if
 * an Angle is inside a certain angle interval.
 * */
class AngleSegment
{
public:
    AngleSegment();

    AngleSegment(const Angle &start, double _width);
    
    /**
     * Tests if the given angle is inside of the segment.
     * @param angle - angle to be tested
     * @return true if angle is inside the segment
     * */
    bool isInside(const Angle &angle) const;

    /**
     * Tests if the given segment is inside of this segment.
     * @param segment - segment to be tested
     * @return true if the given segment is inside of this segment
     * */
    bool isInside(const AngleSegment &segment) const;

    bool split(const Angle &angle, AngleSegment &rest)
    {
        return false;
    }

    std::vector<AngleSegment> split(const Angle &angle)
    {
        return std::vector<AngleSegment>();
    }

    /**
     * Calulates the intersections of this and the given segment.
     * @param b The segment wich schould be tested
     * @return A vector, containing a segment for each intersecting part of the segments.
     * */
    std::vector<AngleSegment> getIntersections(const AngleSegment &b) const;

    /**
     * Returns the width of the segment in radians
     * @return the width in radians
     * */
    double getWidth() const
    {
        return width;
    }
    
    /**
     * Returns the start angle of the segement
     * @return the start angle of the segement
     * */
    base::Angle getStart() const;

    /**
     * Returns the end angle of the segement
     * Note, as the return value is normalized,
     * using it for computation may result in 
     * unexpected behaviour. Better use 
     * getStart + getWidth;
     * @return the end angle of the segement
     * */
    base::Angle getEnd() const;
    
    /**
     * Widht of the segment in radians
     * */
    double width;
    
    /**
     * Start angle of the segment
     * */
    double startRad;

    /**
     * End angle of the segment
     * */
    double endRad;
};

std::ostream& operator << (std::ostream& os, AngleSegment seg);

}

#endif
