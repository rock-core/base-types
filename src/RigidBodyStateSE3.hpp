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

/** Transform of a twist from a coordinate frame A to another coordinate frame B. The mapping is performed using
 *  the adjoint Adj(X) of the given input transform X in SE(3)
 *  @param transform Position and orientation of frame A expressed in frame B (not vice versa!)
 *  @param twist_in Input twist
 *  @returns Twist in new coordinate frame B
 */
Twist operator*(const Pose& transform, const Twist& twist_in);

/** Transform of a spatial acceleration from a coordinate frame A to another coordinate frame B. The mapping is performed using
 *  the adjoint Adj(X) of the given input transform X in SE(3)
 *  @param transform Position and orientation of frame A expressed in frame B (not vice versa!)
 *  @param acc_in Input spatial acceleration
 *  @returns Spatial acceleration in new coordinate frame B
 */
Acceleration operator*(const Pose& transform, const Acceleration& acc_in);

/** Transform of a wrench from a coordinate frame A to another coordinate frame B. The mapping is performed using
 *  the co-adjoint Adj^-T(X) of the given input transform X in SE(3)
 *  @param transform Position and orientation of frame A expressed in frame B (not vice versa!)
 *  @param wrench_in Input wrench
 *  @returns Wrench in new coordinate frame B
 */
Wrench operator*(const Pose& transform, const Wrench& wrench_in);

}

#endif
