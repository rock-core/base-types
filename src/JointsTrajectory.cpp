#include "JointsTrajectory.hpp"

namespace base {

bool JointsTrajectory::isValid() const
{
    size_t samples = getTimeSteps();

    for(size_t i=0; i<elements.size(); ++i)
    {
        if( elements[i].size() != samples )
            return false;
    }

    if( !times.empty() && times.size() != samples )
        return false;

    return true;
}

void JointsTrajectory::resize(int num_joints, int num_samples)
{
    this->resize(num_joints);
    for(size_t i=0; i<elements.size(); i++){
        elements[i].resize(num_samples);
    }
}

void JointsTrajectory::resize(int num_joints)
{
    elements.resize(num_joints);
    names.resize(num_joints);
}

void JointsTrajectory::getJointsAtTimeStep(size_t time_step, samples::Joints& joints)
{
    if(time_step > getTimeSteps())
        throw(InvalidTimeStep(time_step));
        
    joints.resize(getNumberOfJoints());
    joints.names = names;
    for(size_t i=0; i<getNumberOfJoints(); i++){
        joints.elements[i] = elements[i][time_step];
    }
}

bool JointsTrajectory::isTimed() const
{
    return !times.empty();
}

size_t JointsTrajectory::getTimeSteps() const
{
    size_t steps = 0;
    if( !elements.empty() )
        steps = elements[0].size();
    return steps;
}

size_t JointsTrajectory::getNumberOfJoints() const
{
    return elements.size();
}

Time JointsTrajectory::getDuration() const
{
    Time summed;
    for(size_t i=0; i<times.size(); i++)
    {
        summed = summed+times[i];
    }
    return summed;
}

} //end namespace base





