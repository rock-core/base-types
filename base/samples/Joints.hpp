#ifndef BASE_SAMPLES_JOINTS_HPP
#define BASE_SAMPLES_JOINTS_HPP

#include <stdexcept>
#include <vector>

#include <base/Time.hpp>
#include <base/JointState.hpp>

namespace base
{
    namespace samples
    {
        /** Data structure that gives out state readings for a set of joints
         */
        struct Joints
        {
            /** The sample timestamp */
            base::Time time;

            /** The names of the joints described in this structure, in the same
             * order than the 'states' field below
             */
            std::vector<std::string> names;

            /** The joint state information */
            std::vector<JointState> states;

            /** Returns the joint state information for the given joint */
            JointState getStateByName(std::string jointName) const
            {
                for (size_t i = 0; i < names.size(); ++i)
                    if (names[i] == jointName)
                        return states[i];

                throw std::runtime_error(jointName + " is not a joint name contained in this structure");
            }
        };
    }
}

#endif

