#ifndef BASE_TWIST_HPP
#define BASE_TWIST_HPP

#include <base/Eigen.hpp>

namespace base {

/** A twist is represents the velocity of a rigid body as angular velocity around an axis and a linear velocity along this axis */
struct Twist
{
    Twist();
    Twist(base::Vector3d linear, base::Vector3d angular);

    /** Set all members to NaN*/
    void setNaN();
    /** Set all members to zero/Identity*/
    void setZero();
    /** Return false if one of the entries is NaN*/
    bool isValid() const;

    /** Linear 3D velocity in order x-y-z (m/s)*/
    base::Vector3d linear;
    /** Angular 3D velocity in order rot_x-rot_y-rot_z (rad/s)*/
    base::Vector3d angular;
};

/** Component-wise addition of two twists */
Twist operator+(const Twist& a, const Twist& b);
/** Component-wise subtraction of two twists */
Twist operator-(const Twist& a, const Twist& b);
/** Component-wise multiplication of a 6D-Vector with a twist */
Twist operator*(const base::Vector6d& a, const Twist& b);

}

#endif
