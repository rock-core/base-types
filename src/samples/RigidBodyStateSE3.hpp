#ifndef BASE_SAMPLES_RIGID_BODY_STATE_SE3_HPP
#define BASE_SAMPLES_RIGID_BODY_STATE_SE3_HPP

#include <base/RigidBodyStateSE3.hpp>
#include <base/Time.hpp>

namespace base{ namespace samples {

/** RigidBodyStateSE3 with sampled time and frame ID*/
struct RigidBodyStateSE3 : public base::RigidBodyStateSE3
{
    base::Time time;
    /** Coordinate frame in which all quantities are expressed */
    std::string frame_id;
};

}}

#endif
