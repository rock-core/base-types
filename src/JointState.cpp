#include "JointState.hpp"

base::JointState base::JointState::Position(double value)
{
    JointState ret;
    ret.position = value;
    return ret;
}

base::JointState base::JointState::Speed(float value)
{
    JointState ret;
    ret.speed = value;
    return ret;
}

base::JointState base::JointState::Effort(float value)
{
    JointState ret;
    ret.effort = value;
    return ret;
}

base::JointState base::JointState::Raw(float value)
{
    JointState ret;
    ret.raw = value;
    return ret;
}

base::JointState base::JointState::Acceleration(float value)
{
    JointState ret;
    ret.acceleration = value;
    return ret;
}

bool base::JointState::hasPosition() const
{
    return !base::isUnset(position);
}

bool base::JointState::hasSpeed() const
{
    return !base::isUnset(speed);
}

bool base::JointState::hasEffort() const
{
    return !base::isUnset(effort);
}

bool base::JointState::hasRaw() const
{
    return !base::isUnset(raw);
}

bool base::JointState::hasAcceleration() const
{
    return !base::isUnset(acceleration);
}

bool base::JointState::isPosition() const
{
    return hasPosition() && !hasSpeed() && !hasEffort() && !hasRaw() && !hasAcceleration();
}

bool base::JointState::isSpeed() const
{
    return !hasPosition() && hasSpeed() && !hasEffort() && !hasRaw() && !hasAcceleration();
}

bool base::JointState::isEffort() const
{
    return !hasPosition() && !hasSpeed() && hasEffort() && !hasRaw() && !hasAcceleration();
}

bool base::JointState::isRaw() const
{
    return !hasPosition() && !hasSpeed() && !hasEffort() && hasRaw() && !hasAcceleration(); 
}

bool base::JointState::isAcceleration() const
{
    return !hasPosition() && !hasSpeed() && !hasEffort() && !hasRaw() && hasAcceleration();
}

double base::JointState::getField(int mode) const
{
    switch(mode)
    {
        case POSITION: return position;
        case SPEED: return speed;
        case EFFORT: return effort;
        case RAW: return raw;
        case ACCELERATION: return acceleration;
        default: throw std::runtime_error("invalid mode given to getField");
    }
}

void base::JointState::setField(int mode, double value)
{
    switch(mode)
    {
        case POSITION:
            position = value;
            return;
        case SPEED:
            speed = value;
            return;
        case EFFORT:
            effort = value;
            return;
        case RAW:
            raw = value;
            return;
        case ACCELERATION:
            acceleration = value;
            return;
        default: throw std::runtime_error("invalid mode given to getField");
    }
}

base::JointState::MODE base::JointState::getMode() const
{
    if (isPosition())    return POSITION;
    else if (isSpeed())  return SPEED;
    else if (isEffort()) return EFFORT;
    else if (isRaw())    return RAW;
    else if (isAcceleration())    return ACCELERATION;
    else if (hasPosition() || hasSpeed() || hasEffort() || hasRaw() || hasAcceleration())
        throw std::runtime_error("getMode() called on a JointState that has more than one field set");
    else
        return UNSET;
}














