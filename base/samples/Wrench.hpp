#ifndef BASE_SAMPLES_WRENCH_HPP
#define BASE_SAMPLES_WRENCH_HPP

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>

namespace base { namespace samples {

    // Represents the force and torque applied at a point 
    struct Wrench 
    {
        base::Time time;

        /** Force in N */
        base::Vector3d force;

	    /** Torque in Nm*/
        base::Vector3d torque;
    };

    /** 
     * Wrenches with names
    */
    struct Wrenches : public base::NamedVector<Wrench>
    {
        /** The sample timestamp */
        base::Time time;
    };
}}

#endif


