#ifndef BASE_SAMPLES_TWIST_HPP
#define BASE_SAMPLES_TWIST_HPP

#include <base/Time.hpp>
#include <base/Twist.hpp>

namespace base { namespace samples {

/** Twist with sampled time and frame ID*/
struct Twist : public base::Twist{
    base::Time time;
    /** ID of the coordinate frame in which the twist is expressed*/
    std::string frame_id;

};

}}

#endif
