#ifndef __BASE_ANGLE_HH__
#define __BASE_ANGLE_HH__

#include <boost/format.hpp>
#include <math.h>
#include <base/Eigen.hpp>
#include <iostream>
#include <base/Deprecated.hpp>
#include <base/Float.hpp>

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

    /** Use this method to represent an unknown angle
     */
    static inline Angle unknown()
    {
        Angle result;
        result.rad = base::unknown<double>();
        return result;
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

BASE_TYPES_DEPRECATED_SUPPRESS_START
    /**
     * @deprecated this function does not work is right_limit == left_limit.
     *             From the API one can't differenciate between small interval 
     *             or a full cycle. Use base::AngleSegment instead
    * @return true if the angle is insight the given interval
    */
    bool isInRange(const Angle& left_limit, const Angle& right_limit) const BASE_TYPES_DEPRECATED
    {
        if((right_limit-left_limit).rad < 0)
            return !isInRange(right_limit,left_limit);
        if((*this -left_limit).getRad() >= 0 && (right_limit -*this).getRad() >= 0) 
            return true;
        return false;
    }
BASE_TYPES_DEPRECATED_SUPPRESS_STOP

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

static inline Angle operator*( double a, Angle b )
{
    return Angle::fromRad( a * b.getRad() );
}

static inline std::ostream& operator << (std::ostream& os, Angle angle)
{
    os << angle.getRad() << boost::format("[%3.1fdeg]") % angle.getDeg();
    return os;
}

/**
 * This class represents a Segment of a Circle. 
 * This is primary a helper class for doing test if
 * an Angle is inside a certain angle interval.
 * */
class AngleSegment
{
public:
    AngleSegment(): width(0), startRad(0), endRad(0)
    {
    }

    AngleSegment(const Angle &start, double width): width(width), startRad(start.getRad()), endRad(startRad + width)
    {
        if(width < 0)
            throw std::runtime_error("Error got segment with negative width");
    }
    
    /**
     * Tests if the given angle is inside of the segment.
     * @param angle - angle to be tested
     * @return true if angle is inside the segment
     * */
    bool isInside(const Angle &angle) const
    {
        double angleRad = angle.getRad();
        if(angleRad < startRad)
            angleRad += 2*M_PI;
        
        if(angleRad <= endRad) //startRad <= angleRad && 
            return true;
        
        return false;
    };

    /**
     * Tests if the given segment is inside of this segment.
     * @param segment - segment to be tested
     * @return true if the given segment is inside of this segment
     * */
    bool isInside(const AngleSegment &segment) const
    {
        double otherStart = segment.startRad;
        if(otherStart < startRad)
            otherStart += 2*M_PI;

        double otherEnd = otherStart + segment.width;
        
        if(otherEnd <= endRad)
            return true;
        
        return false;
    };

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
    std::vector<AngleSegment> getIntersections(const AngleSegment &b) const
    {
        std::vector<AngleSegment> ret;
        //special case, this segment is a whole circle
        if(width >= 2*M_PI)
        {
            ret.push_back(b);
            return ret;
        }
        
        //special case, other segment is a whole circle
        if(b.width >= 2*M_PI)
        {
            ret.push_back(*this);
            return ret;
        }

        double startA = startRad;
        double startB = b.startRad;
        double widthA = width;
        double widthB = b.width;
        
        //make A the smaller angle
        if(startA > startB)
        {
            std::swap(startA, startB);
            std::swap(widthA, widthB);
        }
        double endA = startA + widthA;
        double endB = startB + widthB;

        //test if segemnts do not intersect at all
        if(endA < startB)
        {
            //wrap case
            if(endB > M_PI)
            {
                //check if segments intersect after wrap correction
                if(startA < endB - 2*M_PI)
                {
                    //this means the start of A is inside of B
                    //drop first part of B and realign it to -M_PI
                    //also switch A and B as B is now the 'lower' one
                    double newWidthA = widthB - (M_PI - startB);
                    startB = startA;
                    widthB = widthA;
                    startA = - M_PI;
                    widthA = newWidthA;
                    endA = startA + widthA;
                    endB = startB + widthB;
                    //no return, still need 
                    //to check for intersection
                }
                else
                    //no intersection
                    return ret;
            } else
                    //no intersection
                return ret;
        }

        //normal case, no wrap around
        double newStart = startB;        
        double newEnd = 0;
        
        if(endA < endB)
        {
            newEnd = endA;
        }
        else
        {
            newEnd = endB;
        }
        
        double newWidth = newEnd - newStart;

        //filter invalid segments
        if(newWidth > 1e-10)
            ret.push_back(AngleSegment(Angle::fromRad(newStart), newWidth));
        
        newStart = endB - 2*M_PI;
        if(newStart > startA)
        {
            newWidth = newStart - startA;
            //filter invalid segments
            if(newWidth > 1e-10)
                ret.push_back(AngleSegment(Angle::fromRad(startA), newWidth));
        }
        
        return ret;
    }

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
    base::Angle getStart() const
    {
        return base::Angle::fromRad(startRad);
    }

    /**
     * Returns the end angle of the segement
     * Note, as the return value is normalized,
     * using it for computation may result in 
     * unexpected behaviour. Better use 
     * getStart + getWidth;
     * @return the end angle of the segement
     * */
    base::Angle getEnd() const
    {
        return base::Angle::fromRad(endRad);
    }
    
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

static inline std::ostream& operator << (std::ostream& os, AngleSegment seg)
{
    os << " Segmend start " << seg.startRad/M_PI *180.0 << " end  " << seg.endRad/M_PI * 180.0 << " width " << seg.width /M_PI * 180.0;
    return os;
}

}

#endif
