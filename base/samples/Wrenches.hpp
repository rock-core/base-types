#ifndef BASE_SAMPLES_WRENCHES_HPP
#define BASE_SAMPLES_WRENCHES_HPP

#include <base/NamedVector.hpp>
#include <base/WrenchState.hpp>
#include <base/Time.hpp>

namespace base { namespace samples {

    /**
     * A named vector of base::WrenchState with sampled Time.
     */
    struct Wrenches : public base::NamedVector< base::WrenchState >
    {
        base::Time time;
    };
}}

#endif


