#ifndef BASE_SAMPLES_ACCELERATION_HPP
#define BASE_SAMPLES_ACCELERATION_HPP

#include <base/Acceleration.hpp>
#include <base/Time.hpp>

namespace base { namespace samples {

/** Spatial Acceleration with sampled time and frame ID*/
struct Acceleration : public base::Acceleration{
    base::Time time;
};

}}

#endif
