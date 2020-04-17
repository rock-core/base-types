#ifndef BASE_ACCELERATION_HPP
#define BASE_ACCELERATION_HPP

#include <base/Eigen.hpp>

namespace base {

/** Spatial acceleration of a rigid body as angular acceleration around an axis and a linear acceleration along this axis */
struct Acceleration
{
    Acceleration();
    Acceleration(base::Vector3d linear, base::Vector3d angular);

    /** Set all members to NaN*/
    void setNaN();
    /** Set all members to zero/Identity*/
    void setZero();
    /** Return false if one of the entries is NaN*/
    bool isValid() const;

    /** Linear 3D acceleration in order x-y-z (m/ss)*/
    base::Vector3d linear;
    /** Angular 3D acceleration in order rot_x-rot_y-rot_z (rad/ss)*/
    base::Vector3d angular;
};

/** Component-wise addition of two spatial accelerations */
Acceleration operator+(const Acceleration& a, const Acceleration& b);
/** Component-wise subtraction of two spatial accelerations */
Acceleration operator-(const Acceleration& a, const Acceleration& b);

}

#endif
