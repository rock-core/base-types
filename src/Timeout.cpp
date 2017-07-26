#include "Timeout.hpp"

namespace base {

Timeout::Timeout(Time timeout) : timeout(timeout) 
{
    start_time = Time::now();
}

void Timeout::restart()
{
    start_time = Time::now();
}

bool Timeout::elapsed() const
{
    return elapsed(timeout);
}

bool Timeout::elapsed(const Time& timeout) const
{
    if(!timeout.isNull())
    {
        return start_time + timeout < Time::now();
    }
    else
    {
        return false;
    }
}

Time Timeout::timeLeft() const
{
    return timeLeft(timeout);
}

Time Timeout::timeLeft(const Time& timeout) const
{
    if(!timeout.isNull())
    {
        return start_time + timeout - Time::now();
    }
    else
    {
        return Time::max();
    }
}


} //end namespace base


