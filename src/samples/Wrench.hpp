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
        /** @meta role logical_time */
        base::Time time;
        /** ID of the coordinate frame in which the wrench is expressed*/
        std::string frame_id;
    };
}}

#endif


