#include "Angle.hpp"
#include <boost/format.hpp>
#include <base/Eigen.hpp>

namespace base {

Angle Angle::vectorToVector(const Vector3d& a, const Vector3d& b)
{
    double dot = a.dot(b);
    double norm = a.norm() * b.norm();
    return fromRad(acos(dot / norm));
}

Angle Angle::vectorToVector(const Vector3d& a, const Vector3d& b, const Vector3d& positive)
{
    double cos = a.dot(b) / (a.norm() * b.norm());

    bool is_positive = (a.cross(b).dot(positive) > 0);
    if (is_positive)
        return fromRad(acos(cos));
    else
        return fromRad(-acos(cos));
}

std::ostream& operator << (std::ostream& os, Angle angle)
{
    os << angle.getRad() << boost::format("[%3.1fdeg]") % angle.getDeg();
    return os;
}

AngleSegment::AngleSegment(): width(0), startRad(0), endRad(0)
{
}

AngleSegment::AngleSegment(const Angle &start, double _width): width(_width), startRad(start.getRad()), endRad(startRad + width)
{
    if(width < 0)
        throw std::runtime_error("Error got segment with negative width");
}

bool AngleSegment::isInside(const Angle& angle) const
{
    double angleRad = angle.getRad();
    if(angleRad < startRad)
        angleRad += 2*M_PI;
    
    if(angleRad <= endRad) //startRad <= angleRad && 
        return true;
    
    return false;
}

bool AngleSegment::isInside(const AngleSegment& segment) const
{
    double otherStart = segment.startRad;
    if(otherStart < startRad)
        otherStart += 2*M_PI;

    double otherEnd = otherStart + segment.width;
    
    if(otherEnd <= endRad)
        return true;
    
    return false;
}

std::vector< AngleSegment > AngleSegment::getIntersections(const AngleSegment& b) const
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

Angle AngleSegment::getStart() const
{
    return Angle::fromRad(startRad);
}

Angle AngleSegment::getEnd() const
{
    return Angle::fromRad(endRad);
}

std::ostream& operator << (std::ostream& os, AngleSegment seg)
{
    os << " Segmend start " << seg.startRad/M_PI *180.0 << " end  " << seg.endRad/M_PI * 180.0 << " width " << seg.width /M_PI * 180.0;
    return os;
}

} //end namespace base
