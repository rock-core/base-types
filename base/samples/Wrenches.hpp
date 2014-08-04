#ifndef BASE_SAMPLES_WRENCHES_HPP
#define BASE_SAMPLES_WRENCHES_HPP

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <base/samples/Wrench.hpp>

namespace base { namespace samples {

    /**
     * A named vector of wrench.
     */
    struct Wrenches : public base::NamedVector< base::samples::Wrench >
    {
    };
}}

#endif


