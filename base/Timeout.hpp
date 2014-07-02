#ifndef BASE_TIMEOUT_HPP
#define BASE_TIMEOUT_HPP

#include <base/Time.hpp>

namespace base{

/** A timeout tracking class
 */
class Timeout {
private:
    base::Time timeout;
    base::Time start_time;

public:
    /**
     * Initializes and starts a timeout
     * @param timeout, if zero is given the timeout is inactive 
     */
    Timeout(base::Time timeout = base::Time::fromSeconds(0))
    : timeout(timeout) 
    {
    start_time = base::Time::now();
    }

    /**
     * Restarts the timeout
     */
    void restart(){
        start_time = base::Time::now();
    }

    /**
     * Checks if the timeout is already elapsed.
     * This uses a syscall, so use sparingly and cache results
     * @returns  true if the timeout is elapsed
     */
    bool elapsed() const{
        return elapsed(timeout);
    }

    /**
     * Checks if the timeout is already elapsed.
     * This uses a syscall, so use sparingly and cache results
     * @param timeout  a custom timeout
     * @returns  true if the timeout is elapsed
     */
    bool elapsed(const base::Time &timeout) const{
        if(!timeout.isNull()){
            return start_time + timeout < base::Time::now();
        }else{
            return false;
        }
    }

    /**
     * Calculates the time left for this timeout
     * This uses a syscall, so use sparingly and cache results
     * @returns  number of milliseconds this timeout as left
     */
    base::Time timeLeft() const{
        return timeLeft(timeout);
    }

    /**
     * Calculates the time left for this timeout
     * This uses a syscall, so use sparingly and cache results
     * @param timeout  a custom timeout
     * @returns  number of milliseconds this timeout as left
     */
    base::Time timeLeft(const base::Time &timeout) const{
        if(!timeout.isNull()){
            return start_time + timeout - base::Time::now();
        }else{
            return base::Time::fromSeconds(0);
        }
    }

};

}

#endif

