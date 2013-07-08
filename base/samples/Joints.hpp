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
            /** Exception thrown when trying to find the index of a joint by
             * name, but the name does not exist
             */
            struct InvalidName : public std::runtime_error
            {
                std::string name;
                InvalidName(std::string const& name)
                    : std::runtime_error("trying to access joint " + name + ", but there is no joint with that name on this Joints structure")
                    , name(name) {}

                ~InvalidName() throw() {}
            };

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
		return states.at( mapNameToIndex( jointName ) );
            }

            /** Resize the states vector to this size
             */
            void resize(size_t size)
            {
                states.resize(size);
                names.resize(size);
            }

            /** Returns the number of joints reported in this structure
             */
            size_t size() const
            {
                return states.size();
            }

	    /** Clears the contents of states and names vector
	     */
	    void clear()
	    {
		states.clear();
		names.clear();
	    }

            /** Returns the joint index that corresponds to the given name
             *
             * @throws InvalidName if the given name does not exist on this
             *   Joints
             */
            size_t mapNameToIndex(std::string const& name) const
            {
                std::vector<std::string>::const_iterator it = find(names.begin(), names.end(), name);
                if (it == names.end())
                    throw InvalidName(name);
                return it - names.begin();
            }
        };
    }
}

#endif

