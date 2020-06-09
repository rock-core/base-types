#ifndef BASE_WRENCH_STATE_HPP
#define BASE_WRENCH_STATE_HPP

#include <base/Eigen.hpp>

namespace base {

    /**
     *  Represents the force and torque applied at a point 
     */
    struct Wrench
    {
        /** Initialize all members with NaN */
        Wrench();
        /** Initialize with the given values*/
        Wrench(base::Vector3d linear, base::Vector3d angular);

        /** Set all members to NaN*/
        void setNaN();
        /** Set all members to zero*/
        void setZero();
        /** Return false if one of the entries is NaN*/
        bool isValid() const;

        /** Force in N */
        base::Vector3d force;

	/** Torque in Nm*/
        base::Vector3d torque;
    };

/** Component-wise addition of two wrenches. This is only meaningful if both are defined in a common reference frame. */
Wrench operator+(const Wrench& a, const Wrench& b);
/** Component-wise subtraction of two wrenches. This is only meaningful if both are defined in a common reference frame. */
Wrench operator-(const Wrench& a, const Wrench& b);
}

#endif


