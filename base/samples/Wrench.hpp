#ifndef BASE_SAMPLES_WRENCH_HPP
#define BASE_SAMPLES_WRENCH_HPP

#include <base/Wrench.hpp>
#include <base/Time.hpp>

namespace base { namespace samples {

    /** 
     * Wrench sample with Force, Torque and sampled Time
     */
    struct Wrench : public base::Wrench
    {
        base::Time time;
    };
}}

#endif


