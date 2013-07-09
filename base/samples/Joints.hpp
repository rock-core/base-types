#ifndef BASE_SAMPLES_JOINTS_HPP
#define BASE_SAMPLES_JOINTS_HPP

#include <stdexcept>
#include <vector>

#include <base/Time.hpp>
#include <base/JointState.hpp>
#include <base/NamedVector.hpp>

namespace base
{
    namespace samples
    {
        /** Data structure that gives out state readings for a set of joints
         */
        struct Joints : public base::NamedVector<JointState>
        {
            /** The sample timestamp */
            base::Time time;
        };
    }
}

#endif

