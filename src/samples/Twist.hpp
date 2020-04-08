#ifndef BASE_SAMPLES_TWIST_HPP
#define BASE_SAMPLES_TWIST_HPP

#include <base/Time.hpp>
#include <base/Twist.hpp>

namespace base { namespace samples {

/** Twist with sampled time */
struct Twist : public base::Twist{
    base::Time time;
};

}}

#endif
