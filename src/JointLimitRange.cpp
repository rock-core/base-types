#include "JointLimitRange.hpp"
#include <sstream>

using namespace base;

std::string JointLimitRange::OutOfBounds::errorString(std::string name, double min, double max, double value)
{
    std::ostringstream ss;
    ss << "The " << name << " value " << value 
        << " was outside the allowed bounds [" << min << ":" << max << "].";
    return ss.str();
}
    
void JointLimitRange::validate(const JointState& state) const
{
    std::pair<bool, OutOfBounds> check = verifyValidity(state);
    if (!check.first)
        throw check.second;
}

bool JointLimitRange::isValid(const JointState& state) const
{
    std::pair<bool, OutOfBounds> check = verifyValidity(state);
    return check.first;
}

JointLimitRange JointLimitRange::Position(double min, double max)
{
    JointLimitRange result;
    result.min.position = min;
    result.max.position = max;
    return result;
}


JointLimitRange JointLimitRange::Speed(double min, double max)
{
    JointLimitRange result;
    result.min.speed = min;
    result.max.speed = max;
    return result;
}


JointLimitRange JointLimitRange::Effort(double min, double max)
{
    JointLimitRange result;
    result.min.effort = min;
    result.max.effort = max;
    return result;
}

JointLimitRange JointLimitRange::Raw(double min, double max)
{
    JointLimitRange result;
    result.min.raw = min;
    result.max.raw = max;
    return result;
}

JointLimitRange JointLimitRange::Acceleration(double min, double max)
{
    JointLimitRange result;
    result.min.acceleration = min;
    result.max.acceleration = max;
    return result;
}

std::pair< bool, JointLimitRange::OutOfBounds > JointLimitRange::verifyValidity(const JointState& state) const
{
    using std::make_pair;
    if( state.hasPosition() )
    {
        if( min.hasPosition() && min.position > state.position )
            return make_pair(false, OutOfBounds( "position", min.position, max.position, state.position ));
        if( max.hasPosition() && max.position < state.position )
            return make_pair(false, OutOfBounds( "position", min.position, max.position, state.position ));
    }

    if( state.hasSpeed() )
    {
        if( min.hasSpeed() && min.speed > state.speed )
            return make_pair(false, OutOfBounds( "speed", min.speed, max.speed, state.speed ));
        if( max.hasSpeed() && max.speed < state.speed )
            return make_pair(false, OutOfBounds( "speed", min.speed, max.speed, state.speed ));
    }

    if( state.hasEffort() )
    {
        if( min.hasEffort() && min.effort > state.effort )
            return make_pair(false, OutOfBounds( "effort", min.effort, max.effort, state.effort ));
        if( max.hasEffort() && max.effort < state.effort )
            return make_pair(false, OutOfBounds( "effort", min.effort, max.effort, state.effort ));
    }

    if( state.hasRaw() )
    {
        if( min.hasRaw() && min.raw > state.raw )
            return make_pair(false, OutOfBounds( "raw", min.raw, max.raw, state.raw ));
        if( max.hasRaw() && max.raw < state.raw )
            return make_pair(false, OutOfBounds( "raw", min.raw, max.raw, state.raw ));
    }

    if( state.hasAcceleration() )
    {
        if( min.hasAcceleration() && min.acceleration > state.acceleration )
            return make_pair(false, OutOfBounds( "acceleration", min.acceleration, max.acceleration, state.acceleration ));
        if( max.hasAcceleration() && max.acceleration < state.acceleration )
            return make_pair(false, OutOfBounds( "acceleration", min.acceleration, max.acceleration, state.acceleration ));
    }
    return make_pair(true, OutOfBounds());
}

std::pair<bool, JointState> JointLimitRange::saturate(const JointState& state) const
{
    JointState new_state = state;
    bool saturated = false;
    if (state.hasPosition()) {
        if (min.hasPosition() && min.position > state.position) {
            new_state.position = min.position;
            saturated = true;
        }
        if (max.hasPosition() && max.position < state.position) {
            new_state.position = max.position;
            saturated = true;
        }
    }

    if (state.hasSpeed()) {
        if (min.hasSpeed() && min.speed > state.speed) {
            new_state.speed = min.speed;
            saturated = true;
        }
        if (max.hasSpeed() && max.speed < state.speed) {
            new_state.speed = max.speed;
            saturated = true;
        }
    }

    if (state.hasEffort()) {
        if (min.hasEffort() && min.effort > state.effort) {
            new_state.effort = min.effort;
            saturated = true;
        }
        if (max.hasEffort() && max.effort < state.effort) {
            new_state.effort = max.effort;
            saturated = true;
        }
    }

    if (state.hasRaw()) {
        if (min.hasRaw() && min.raw > state.raw) {
            new_state.raw = min.raw;
            saturated = true;
        }
        if (max.hasRaw() && max.raw < state.raw) {
            new_state.raw = max.raw;
            saturated = true;
        }
    }

    if (state.hasAcceleration()) {
        if (min.hasAcceleration() && min.acceleration > state.acceleration) {
            new_state.acceleration = min.acceleration;
            saturated = true;
        }
        if (max.hasAcceleration() && max.acceleration < state.acceleration) {
            new_state.acceleration = max.acceleration;
            saturated = true;
        }
    }
    return std::make_pair(saturated, new_state);
}
