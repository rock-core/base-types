#ifndef BASE_RIGID_BODY_STATE_SE3_HPP
#define BASE_RIGID_BODY_STATE_SE3_HPP

#include <base/Pose.hpp>
#include <base/Twist.hpp>
#include <base/Acceleration.hpp>
#include <base/Wrench.hpp>

namespace base {

/** Representation of the complete state of a rigid body in SE3 (Special Euclidean Group)*/
struct RigidBodyStateSE3
{
    /** Initialize all members with NaN */
    RigidBodyStateSE3(){
        setNaN();
    }
    /** Set all members to NaN*/
    void setNaN();
    /** Check if all pose entries are valid, i.e. not NaN and if the orientation is a unit quaternion*/
    bool hasValidPose() const;
    /** Check if all twist entries are valid, i.e. not NaN*/
    bool hasValidTwist() const;
    /** Check if all acceleration entries are valid, i.e. not NaN*/
    bool hasValidAcceleration() const;
    /** Check if all wrench entries are valid, i.e. not NaN*/
    bool hasValidWrench() const;

    /** 3D position and orientation of the body*/
    Pose pose;
    /** 3D Linear and angular velocity of the body*/
    Twist twist;
    /** 3D Linear and angular acceleration of the body*/
    Acceleration acceleration;
    /** Force and torque acting on the body*/
    Wrench wrench;
};

/** Transform of a twist \f$ V = (\omega,v)^T \f$ from a coordinate frame A to another coordinate frame B. The mapping is performed using
 *  the adjoint \f$ Adj(X) \in R^{6 \times 6} \f$ of the given input transform \f$X = (R,p) \in SE(3)\f$ as follows*:
 *  \f[
 *     \left(\begin{array}{cc} \omega \\ v \end{array}\right)_B = \left(\begin{array}{cc} R & 0 \\ \left[p\right]R & R \end{array}\right) \left(\begin{array}{cc} \omega \\ v \end{array}\right)_A
 *  \f]
 * with \n
 * \f[
 *     \left[p\right] = \left(\begin{array}{ccc}0 & -p_z & p_y \\ p_z & 0 &-p_x \\ -p_y & p_x & 0\end{array}\right)
 * \f]
 *  and \n
 *  \f$ \omega \in R^3\f$ - Angular velocity \n
 *  \f$ v \in R^3\f$ - Linear velocity \n
 *  \f$ R \in SO(3)\f$ - Rotation matrix \n
 *  \f$ p \in R^3\f$ - Translation vector \n\n
 *
 *  *According to: Lynch, K.M. and Park, F.C. 2017. Modern Robotics: Mechanics, Planning, and Control. page 100. Cambridge University Press, USA
 *  @param transform Input transform as position and orientation of frame A expressed in frame B (not vice versa!)
 *  @param twist_in Input twist, expressed in coordinate frame A
 *  @returns Twist in new coordinate frame B
 */
Twist operator*(const Pose& transform, const Twist& twist_in);

/** Transform of a spatial acceleration from a coordinate frame A to another coordinate frame B. The mapping is performed using
 *  the adjoint \f$ Adj(X) \in R^{6 \times 6} \f$ of the given input transform \f$X = (R,p) \in SE(3)\f$ in the same manner as for the twist operator.
 *  @param transform Input transform as position and orientation of frame A expressed in frame B (not vice versa!)
 *  @param acc_in Input spatial acceleration, expressed in coordinate frame A
 *  @returns Spatial acceleration in new coordinate frame B
 */
Acceleration operator*(const Pose& transform, const Acceleration& acc_in);

/** Transform of a wrench \f$ F = (m,f)^T \f$ from a coordinate frame A to another coordinate frame B. The mapping is performed using
 *  the co-adjoint \f$ Adj(X)^{-T} \in R^{6 \times 6} \f$ of the given input transform \f$X = (R,p) \in SE(3)\f$ as follows*:
 *  \f[
 *     \left(\begin{array}{cc} m \\ f \end{array}\right)_B = \left(\begin{array}{cc} R & \left[p\right]R \\ 0 & R \end{array}\right) \left(\begin{array}{cc} m \\ f \end{array}\right)_A
 *  \f]
 * with \n
 * \f[
 *     \left[p\right] = \left(\begin{array}{ccc}0 & -p_z & p_y \\ p_z & 0 &-p_x \\ -p_y & p_x & 0\end{array}\right)
 * \f]
 *  and \n
 *  \f$ m \in R^3\f$ - Torque/moment \n
 *  \f$ f \in R^3\f$ - Linear force \n
 *  \f$ R \in SO(3)\f$ - Rotation matrix \n
 *  \f$ p \in R^3\f$ - Translation vector \n\n
 *
 *  *According to: Lynch, K.M. and Park, F.C. 2017. Modern Robotics: Mechanics, Planning, and Control. page 110. Cambridge University Press, USA
 *  @param transform Input transform as position and orientation of frame A expressed in frame B (not vice versa!)
 *  @param twist_in Input twist, expressed in coordinate frame A
 *  @returns Twist in new coordinate frame B
 */
Wrench operator*(const Pose& transform, const Wrench& wrench_in);

}

#endif
