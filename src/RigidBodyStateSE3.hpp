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
    RigidBodyStateSE3(){
        setNaN();
    }
    /** Set all members to NaN*/
    void setNaN();
    /** Check if the pose is valid, e.g. not NaN*/
    bool hasValidPose() const;
    /** Check if the twist is valid, e.g. not NaN*/
    bool hasValidTwist() const;
    /** Check if the acceleration is valid, e.g. not NaN*/
    bool hasValidAcceleration() const;
    /** Check if the wrench is valid, e.g. not NaN*/
    bool hasValidWrench() const;

    /** 3D position and orientation*/
    Pose pose;
    /** 3D Linear and angular velocity*/
    Twist twist;
    /** 3D Linear and angular acceleration*/
    Acceleration acceleration;
    /** Force and torque*/
    Wrench wrench;
};

/** Change reference coordinate system of a twist.
 *  @param transform Transformation to new coordinate system
 *  @param twist_in Input twist
 *  @returns Twist in transformed coordinate system
 */
Twist operator*(const Pose& transform, const Twist& twist_in);

/** Change reference coordinate system of a spatial acceleration.
 *  @param transform Transformation to new coordinate system
 *  @param acc_in Input acceleration
 *  @returns Spatial acceleration in new coordinate system
 */
Acceleration operator*(const Pose& transform, const Acceleration& acc_in);

/** Change reference coordinate system of a wrench.
 *  @param transform Transformation to new coordinate system
 *  @param wrench_in Input wrench
 *  @returns Wrench in new coordinate system
 */
Wrench operator*(const Pose& transform, const Wrench& wrench_in);

}

#endif
