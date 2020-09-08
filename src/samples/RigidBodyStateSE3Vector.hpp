#ifndef BASE_SAMPLES_RIGID_BODY_STATE_SE3_VECTOR_HPP
#define BASE_SAMPLES_RIGID_BODY_STATE_SE3_VECTOR_HPP

#include <base/NamedVector.hpp>
#include <base/Time.hpp>
#include <base/RigidBodyStateSE3.hpp>

namespace base { namespace samples {

/** Named vector of base::RigidBodyStateSE3 with sampled Time. */
struct RigidBodyStateSE3Vector : public base::NamedVector< base::RigidBodyStateSE3>
{
    base::Time time;
    /** Common coordinate frame for all elements of the vector */
    std::string frame_id;
};

}}

#endif
