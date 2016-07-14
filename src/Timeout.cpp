#include "Timeout.hpp"

base::Timeout::Timeout(base::Time timeout) : timeout(timeout) 
{
    start_time = base::Time::now();
}

void base::Timeout::restart()
{
    start_time = base::Time::now();
}

bool base::Timeout::elapsed() const
{
    return elapsed(timeout);
}

bool base::Timeout::elapsed(const base::Time& timeout) const
{
    if(!timeout.isNull())
    {
        return start_time + timeout < base::Time::now();
    }
    else
    {
        return false;
    }
}

base::Time base::Timeout::timeLeft() const
{
    return timeLeft(timeout);
}

base::Time base::Timeout::timeLeft(const base::Time& timeout) const
{
    if(!timeout.isNull())
    {
        return start_time + timeout - base::Time::now();
    }
    else
    {
        return base::Time::fromSeconds(0);
    }
}





