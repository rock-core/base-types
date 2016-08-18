#include "JointLimits.hpp"

namespace base {

bool JointLimits::isValid(const samples::Joints& joints) const
{
    if (joints.hasNames())
    {
        for( size_t i=0; i<joints.size(); i++ )
            if (! (*this)[joints.names[i]].isValid( joints[i] ))
                return false;
    }
    else
    {
        for( size_t i=0; i<joints.size(); i++ )
            if (! (*this)[i].isValid( joints[i] ))
                return false;
    }
    return true;
}

void JointLimits::validate(const samples::Joints& joints) const
{
    if (joints.hasNames())
    {
        for( size_t i=0; i<joints.size(); i++ )
            (*this)[joints.names[i]].validate( joints[i] );
    }
    else
    {
        for( size_t i=0; i<joints.size(); i++ )
            (*this)[i].validate( joints[i] );
    }
}

} //end namespace base