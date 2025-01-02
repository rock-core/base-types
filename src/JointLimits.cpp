#include "JointLimits.hpp"

using namespace base;

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

std::pair<bool, samples::Joints> JointLimits::saturate(const samples::Joints& joints)
{
    samples::Joints new_joints;
    new_joints.resize(joints.size());
    new_joints = joints;
    bool saturated = false;
    if (joints.hasNames()) {
        for (size_t i = 0; i < joints.size(); i++)
        {
            auto result = (*this)[joints.names[i]].saturate(joints[i]);
            saturated = saturated ? saturated : result.first;
            new_joints[i] = result.second;
        }
    }
    else {
        for (size_t i = 0; i < joints.size(); i++)
        {
            auto result = (*this)[i].saturate(joints[i]);
            saturated = saturated ? saturated : result.first;
            new_joints[i] = result.second;
        }
    }
    return std::make_pair(saturated, new_joints);
}
