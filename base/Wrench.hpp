#ifndef BASE_WRENCH_STATE_HPP
#define BASE_WRENCH_STATE_HPP

#include <base/Eigen.hpp>

namespace base {

    /**
     *  Represents the force and torque applied at a point 
     */
    struct Wrench
    {
        /** Force in N */
        base::Vector3d force;

	/** Torque in Nm*/
        base::Vector3d torque;
    };
}

#endif


